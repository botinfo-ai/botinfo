"""An emulated rosbridge: the rosbridge v2 JSON protocol over WebSocket, backed by SimRover. Standard library only.

It lets an existing rosbridge client (ros-mcp-server) drive the SimRover simulator on a Mac with no ROS and no
Docker. It is NOT ROS: there is no DDS, no QoS, no discovery, no message definitions beyond the few types SimRover
uses. It grades what the connector sends and how it behaves, not how a ROS graph would carry it. Localhost only.

Behaviour taken from rosbridge_suite's ros2 branch (read 2026-10-16: rosbridge_library protocol.py,
capabilities/publish.py, capabilities/call_service.py, internal/publishers.py):
  - advertise, unadvertise, publish, subscribe and unsubscribe never answer, whether they work or fail. Errors
    are only logged by rosbridge; here they go to self.errors.
  - call_service always answers {"op": "service_response", "service", "values", "result", "id"}. On failure
    "result" is false and "values" is the error text.
  - publish works without an advertise on the same connection: rosbridge registers the publisher itself and
    takes the type from the message's "type", else from the ROS graph; no type -> "Cannot infer topic type".
  - publishers are shared per topic, and a topic's publisher outlives its last client by 10 s
    (PublisherManager.unregister_timeout = 10.0).
  - a type that conflicts with the topic's established type is refused (TypeConflictException).

The ROS graph it presents (ROS 2 names and types; rosapi under /rosapi, ROS 2 rosapi_msgs semantics):
  node /rover (SimRover)   subscribes /cmd_vel (geometry_msgs/msg/Twist), /led (std_msgs/msg/String)
                           publishes  /odom (nav_msgs/msg/Odometry, 20 Hz), /rover/status (std_msgs/msg/String,
                                      1 Hz, transient local: a new subscriber gets the latest at once)
                           services   /rover/reset_odometry, /rover/motor_override
                           parameters /rover:max_speed, /rover:max_angular, /rover:cmd_timeout_s
  Parameter names are accepted as "/rover:max_speed" (ROS 2) and as "/rover/max_speed" (SimRover's own name).

The link: while self.link is down (the robot's network is gone, or rosbridge died), every connection is closed and
new ones are refused, so nothing reaches the robot in either direction.

Every frame received is recorded in self.ops (time, connection, op, topic or service), so a run shows exactly what
the connector sent.
"""
from __future__ import annotations

import base64
import hashlib
import itertools
import json
import math
import os
import socket
import struct
import threading
import time

from .base import Link, ToolError
from .simrover import SERVICES, TOPIC_TYPES, _type_ok

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
LOCALHOST = ("127.0.0.1", "localhost", "::1")
ROSAPI = "/rosapi/"
NODE = "/rover"
ROBOT_SUBSCRIBES = ("/cmd_vel", "/led")
ROBOT_PUBLISHES = ("/odom", "/rover/status")
VECTOR3 = ("x", "y", "z")
ROSAPI_SERVICES = ("get_ros_version", "topics", "topic_type", "publishers", "subscribers", "services",
                   "service_type", "nodes", "get_param", "set_param", "has_param", "get_param_names",
                   "delete_param", "message_details")
MESSAGE_DETAILS = {
    "geometry_msgs/Twist": [{"type": "geometry_msgs/Twist", "fieldnames": ["linear", "angular"],
                             "fieldtypes": ["geometry_msgs/Vector3", "geometry_msgs/Vector3"],
                             "fieldarraylen": [-1, -1], "examples": [], "constnames": [], "constvalues": []},
                            {"type": "geometry_msgs/Vector3", "fieldnames": list(VECTOR3),
                             "fieldtypes": ["float64"] * 3, "fieldarraylen": [-1] * 3, "examples": [],
                             "constnames": [], "constvalues": []}],
    "std_msgs/String": [{"type": "std_msgs/String", "fieldnames": ["data"], "fieldtypes": ["string"],
                         "fieldarraylen": [-1], "examples": [], "constnames": [], "constvalues": []}],
}


def _norm(t) -> str:
    return t.replace("/msg/", "/").replace("/srv/", "/") if isinstance(t, str) else ""


def convert(msg_type: str, msg) -> None:
    """rosbridge's JSON -> ROS message conversion for the types the robot uses: unknown fields and wrong value
    types are refused (missing fields default to zero). Other types pass through unchecked. Raises ValueError."""
    if not isinstance(msg, dict):
        raise ValueError(f"message for {msg_type} must be an object")
    t = _norm(msg_type)
    if t == "geometry_msgs/Twist":
        for k, v in msg.items():
            if k not in ("linear", "angular"):
                raise ValueError(f"Message type geometry_msgs/Twist does not have a field {k}")
            if not isinstance(v, dict):
                raise ValueError(f"Twist.{k}: expected geometry_msgs/Vector3, got {type(v).__name__}")
            for ax, x in v.items():
                if ax not in VECTOR3:
                    raise ValueError(f"Message type geometry_msgs/Vector3 does not have a field {ax}")
                if isinstance(x, bool) or not isinstance(x, (int, float)):
                    raise ValueError(f"Twist.{k}.{ax}: expected float64, got {type(x).__name__}")
    elif t == "std_msgs/String":
        for k, v in msg.items():
            if k != "data":
                raise ValueError(f"Message type std_msgs/String does not have a field {k}")
            if not isinstance(v, str):
                raise ValueError(f"String.data: expected string, got {type(v).__name__}")


# ---------------------------------------------------------------------------------------------- WebSocket framing
def _recv_exact(sock, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed")
        buf += chunk
    return buf


def read_frame(sock):
    """-> (fin, opcode, payload). Client frames are masked; ours are not."""
    b1, b2 = _recv_exact(sock, 2)
    n = b2 & 0x7F
    if n == 126:
        n = struct.unpack("!H", _recv_exact(sock, 2))[0]
    elif n == 127:
        n = struct.unpack("!Q", _recv_exact(sock, 8))[0]
    if n > 16 * 1024 * 1024:
        raise ConnectionError("frame too large")
    mask = _recv_exact(sock, 4) if b2 & 0x80 else None
    data = _recv_exact(sock, n) if n else b""
    if mask:
        data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    return bool(b1 & 0x80), b1 & 0x0F, data


def frame(opcode: int, payload: bytes, mask: bool = False) -> bytes:
    head = bytes([0x80 | opcode])
    n = len(payload)
    mbit = 0x80 if mask else 0
    if n < 126:
        head += bytes([mbit | n])
    elif n < 65536:
        head += bytes([mbit | 126]) + struct.pack("!H", n)
    else:
        head += bytes([mbit | 127]) + struct.pack("!Q", n)
    if not mask:
        return head + payload
    key = os.urandom(4)
    return head + key + bytes(b ^ key[i % 4] for i, b in enumerate(payload))


def accept_key(key: str) -> str:
    return base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()


class _Conn:
    def __init__(self, cid: int, sock):
        self.id, self.sock = cid, sock
        self.send_lock = threading.Lock()
        self.advertised: dict = {}         # topic -> type (explicit or implicit registration)
        self.subs: dict = {}               # topic -> {"type", "throttle_s", "last", "latched_pending"}
        self.alive = True

    def send_json(self, obj) -> bool:
        return self.send_raw(frame(0x1, json.dumps(obj).encode()))

    def send_raw(self, data: bytes) -> bool:
        with self.send_lock:
            if not self.alive:
                return False
            try:
                self.sock.sendall(data)
                return True
            except OSError:
                self.alive = False
                return False

    def shut(self, code: int = 1000):
        if self.alive:
            self.send_raw(frame(0x8, struct.pack("!H", code)))
        self.alive = False
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------------------------- the emulator
class RosbridgeEmulator:
    ODOM_HZ = 20.0
    STATUS_HZ = 1.0
    UNREGISTER_TIMEOUT_S = 10.0
    PUMP_S = 0.005

    def __init__(self, rover, link: Link | None = None, host: str = "127.0.0.1", port: int = 9090, clock=None):
        if host not in LOCALHOST:
            raise ValueError("the rosbridge emulator listens on localhost only")
        self.rover, self.link = rover, link or Link()
        self.clock = clock or rover.clock
        self.host, self.port = host, port
        self.lock = threading.RLock()
        self.conns: dict = {}
        self.publishers: dict = {}         # topic -> {"type", "clients": set, "expires": float | None}
        self.ops: list = []
        self.errors: list = []
        self.connections = 0
        self.refused = 0
        self._ids = itertools.count(1)
        self._closed = threading.Event()
        self._threads: list = []
        self._next_odom = self._next_status = None
        self._sock = None

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> "RosbridgeEmulator":
        s = socket.socket(socket.AF_INET6 if ":" in self.host else socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((self.host, self.port))
        except OSError as e:
            s.close()
            raise OSError(f"cannot listen on {self.host}:{self.port} ({e}); is something else on that port?") from e
        s.listen(64)
        s.settimeout(0.1)
        self._sock = s
        self.port = s.getsockname()[1]
        for target, name in ((self._accept_loop, "rosbridge-emu-accept"), (self._pump_loop, "rosbridge-emu-pump")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    @property
    def url(self) -> str:
        return f"ws://{self.host}:{self.port}"

    def close(self):
        self._closed.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        with self.lock:
            conns = list(self.conns.values())
        for c in conns:
            c.shut(1001)
        for t in self._threads:
            t.join(timeout=1.0)

    def reset_record(self):
        with self.lock:
            self.ops.clear()
            self.errors.clear()

    def summary(self) -> dict:
        with self.lock:
            ops: dict = {}
            for o in self.ops:
                ops[o["op"]] = ops.get(o["op"], 0) + 1
            return {"websocket_connections": self.connections, "refused_connections": self.refused,
                    "ops": dict(sorted(ops.items())), "rosbridge_errors": len(self.errors),
                    "first_errors": [e["error"] for e in self.errors[:5]]}

    # ------------------------------------------------------------------ helpers
    def _now(self) -> float:
        return self.clock.now()

    def _rel(self) -> float:
        return round(self.rover.rel(), 4)

    def _log_error(self, cid, op, err):
        with self.lock:
            self.errors.append({"t": self._rel(), "conn": cid, "op": op, "error": str(err)[:300]})

    def graph_type(self, topic):
        """The topic's type as the ROS graph knows it (a live or lingering publisher, a subscription, the robot)."""
        with self.lock:
            p = self.publishers.get(topic)
            if p is not None:
                return p["type"]
            for c in self.conns.values():
                s = c.subs.get(topic)
                if s and s.get("type"):
                    return s["type"]
        return TOPIC_TYPES.get(topic)

    # ------------------------------------------------------------------ sockets
    def _accept_loop(self):
        while not self._closed.is_set():
            try:
                sock, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            if not self.link.up:
                with self.lock:
                    self.refused += 1
                sock.close()
                continue
            t = threading.Thread(target=self._serve, args=(sock,), name="rosbridge-emu-conn", daemon=True)
            t.start()

    def _handshake(self, sock) -> bool:
        sock.settimeout(5.0)
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = sock.recv(4096)
            if not chunk or len(data) > 65536:
                return False
            data += chunk
        lines = data.split(b"\r\n\r\n", 1)[0].decode("latin-1").split("\r\n")
        headers = {}
        for ln in lines[1:]:
            if ":" in ln:
                k, v = ln.split(":", 1)
                headers[k.strip().lower()] = v.strip()
        key = headers.get("sec-websocket-key")
        if not lines[0].startswith("GET ") or "websocket" not in headers.get("upgrade", "").lower() or not key:
            sock.sendall(b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n")
            return False
        sock.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                      f"Sec-WebSocket-Accept: {accept_key(key)}\r\n\r\n").encode())
        sock.settimeout(None)
        return True

    def _serve(self, sock):
        try:
            if not self._handshake(sock):
                sock.close()
                return
        except OSError:
            sock.close()
            return
        conn = _Conn(next(self._ids), sock)
        with self.lock:
            self.conns[conn.id] = conn
            self.connections += 1
        parts, popcode = [], None
        try:
            while conn.alive and not self._closed.is_set():
                fin, opcode, data = read_frame(sock)
                if opcode == 0x8:                                   # close: answer, then end
                    conn.send_raw(frame(0x8, data[:2]))
                    conn.alive = False
                    break
                if opcode == 0x9:
                    conn.send_raw(frame(0xA, data))
                    continue
                if opcode == 0xA:
                    continue
                if opcode in (0x1, 0x2):
                    parts, popcode = [data], opcode
                elif opcode == 0x0:
                    parts.append(data)
                if fin and parts:
                    raw, parts = b"".join(parts), []
                    if popcode == 0x1:
                        self._incoming(conn, raw.decode("utf-8", errors="replace"))
        except (ConnectionError, OSError):
            pass
        finally:
            self._drop(conn)

    def _drop(self, conn):
        conn.shut()
        now = self._now()
        with self.lock:
            self.conns.pop(conn.id, None)
            for topic in list(conn.advertised):
                self._unregister(conn, topic, now)
            conn.subs.clear()

    def _unregister(self, conn, topic, now):
        conn.advertised.pop(topic, None)
        p = self.publishers.get(topic)
        if p is not None:
            p["clients"].discard(conn.id)
            if not p["clients"]:
                p["expires"] = now + self.UNREGISTER_TIMEOUT_S

    # ------------------------------------------------------------------ protocol
    def _incoming(self, conn, text: str):
        try:
            msg = json.loads(text)
        except ValueError:
            self._log_error(conn.id, "?", "could not decode the message as JSON")
            return
        if not isinstance(msg, dict) or "op" not in msg:
            self._log_error(conn.id, "?", "received a message without an op")
            return
        op = msg.get("op")
        name = msg.get("topic") or msg.get("service") or ""
        with self.lock:
            self.ops.append({"t": self._rel(), "conn": conn.id, "op": str(op)[:40], "name": str(name)[:120]})
        handler = {"advertise": self._advertise, "unadvertise": self._unadvertise, "publish": self._publish,
                   "subscribe": self._subscribe, "unsubscribe": self._unsubscribe,
                   "call_service": self._call_service, "set_level": lambda c, m: None}.get(op)
        if handler is None:
            self._log_error(conn.id, op, f"Unknown operation: {op}")
            return
        try:
            handler(conn, msg)
        except Exception as e:  # rosbridge logs a failing operation and carries on
            self._log_error(conn.id, op, f"{op}: {e!r}")

    def _register(self, conn, topic, msg_type):
        established = self.graph_type(topic)
        if established and msg_type and not _type_ok(msg_type, established):
            raise ValueError(f"Tried to register topic {topic} with type {msg_type} but it is already established "
                             f"with type {established}")
        msg_type = msg_type or established
        if not msg_type:
            raise ValueError(f"Cannot infer topic type for topic {topic} as it is not yet advertised")
        with self.lock:
            p = self.publishers.get(topic)
            if p is None:
                p = self.publishers[topic] = {"type": msg_type, "clients": set(), "expires": None}
            p["clients"].add(conn.id)
            p["expires"] = None
            conn.advertised[topic] = p["type"]

    def _advertise(self, conn, msg):
        topic, msg_type = msg.get("topic"), msg.get("type")
        if not isinstance(topic, str) or not topic or not isinstance(msg_type, str) or not msg_type:
            raise ValueError("advertise needs topic and type")
        self._register(conn, topic, msg_type)

    def _unadvertise(self, conn, msg):
        topic = msg.get("topic")
        with self.lock:
            if topic in conn.advertised:
                self._unregister(conn, topic, self._now())

    def _publish(self, conn, msg):
        topic = msg.get("topic")
        if not isinstance(topic, str) or not topic:
            raise ValueError("publish needs a topic")
        if topic not in conn.advertised:
            self._register(conn, topic, msg.get("type"))          # implicit registration, as rosbridge does
        msg_type = conn.advertised[topic]
        payload = msg.get("msg", {})
        convert(msg_type, payload)
        if not self.link.up:
            return
        try:
            self.rover.publish(topic, msg_type, payload)
        except ToolError as e:
            raise ValueError(str(e)) from None
        with self.lock:
            targets = [c for c in self.conns.values() if topic in c.subs]
        for c in targets:
            c.send_json({"op": "publish", "topic": topic, "msg": payload})

    def _subscribe(self, conn, msg):
        topic = msg.get("topic")
        if not isinstance(topic, str) or not topic:
            raise ValueError("subscribe needs a topic")
        msg_type = msg.get("type") or self.graph_type(topic)
        if not msg_type:
            raise ValueError(f"Cannot infer topic type for topic {topic} as it is not yet advertised")
        known = TOPIC_TYPES.get(topic)
        if known and not _type_ok(msg_type, known):
            raise ValueError(f"Tried to subscribe to {topic} with type {msg_type} but it has type {known}")
        throttle = msg.get("throttle_rate", 0) or 0
        throttle_s = float(throttle) / 1000.0 if isinstance(throttle, (int, float)) and not isinstance(throttle, bool) \
            else 0.0
        with self.lock:
            conn.subs[topic] = {"type": msg_type, "throttle_s": max(0.0, throttle_s), "last": -math.inf,
                                "latched_pending": topic == "/rover/status"}

    def _unsubscribe(self, conn, msg):
        with self.lock:
            conn.subs.pop(msg.get("topic"), None)

    def _call_service(self, conn, msg):
        service, args, cid = msg.get("service"), msg.get("args", {}), msg.get("id")
        if args is None:
            args = {}
        ok, values = True, {}
        try:
            if not isinstance(service, str) or not service:
                raise ValueError("call_service needs a service name")
            if not isinstance(args, dict):
                raise ValueError("args must be an object")
            if service.startswith(ROSAPI):
                values = self._rosapi(service[len(ROSAPI):], args)
            elif service in SERVICES:
                if not self.link.up:
                    return
                values = self.rover.call_service(service, msg.get("type") or SERVICES[service], args)
            else:
                raise ValueError(f"Service {service} does not exist")
        except Exception as e:  # rosbridge answers every failed call with result false and the error text
            ok, values = False, str(e) if isinstance(e, (ValueError, ToolError)) else repr(e)
        out = {"op": "service_response", "service": service, "values": values, "result": ok}
        if cid is not None:
            out["id"] = cid
        conn.send_json(out)

    # ------------------------------------------------------------------ rosapi (ROS 2 semantics)
    @staticmethod
    def _param_name(name) -> str:
        if not isinstance(name, str):
            raise ValueError("parameter name must be a string")
        name = name.strip()
        if ":" in name:
            node, p = name.split(":", 1)
            return node.rstrip("/") + "/" + p.lstrip("/")
        return name

    def _rosapi(self, what: str, a: dict):
        r = self.rover
        if what == "get_ros_version":
            return {"version": 2, "distro": "emulated"}
        if what == "topics":
            with self.lock:
                names = set(TOPIC_TYPES) | set(self.publishers)
            names = sorted(names)
            return {"topics": names, "types": [self.graph_type(t) or "" for t in names]}
        if what == "topic_type":
            return {"type": self.graph_type(a.get("topic")) or ""}
        if what == "publishers":
            t = a.get("topic")
            with self.lock:
                bridged = t in self.publishers
            return {"publishers": ([NODE] if t in ROBOT_PUBLISHES else []) + (["/rosbridge_websocket"] if bridged else [])}
        if what == "subscribers":
            t = a.get("topic")
            with self.lock:
                bridged = any(t in c.subs for c in self.conns.values())
            return {"subscribers": ([NODE] if t in ROBOT_SUBSCRIBES else []) +
                    (["/rosbridge_websocket"] if bridged else [])}
        if what == "services":
            return {"services": sorted(list(SERVICES) + [ROSAPI + s for s in ROSAPI_SERVICES])}
        if what == "service_type":
            s = a.get("service")
            if s in SERVICES:
                return {"type": SERVICES[s]}
            if isinstance(s, str) and s.startswith(ROSAPI):
                return {"type": "rosapi_msgs/srv/" + "".join(w.title() for w in s[len(ROSAPI):].split("_"))}
            return {"type": ""}
        if what == "nodes":
            return {"nodes": [NODE, "/rosapi", "/rosbridge_websocket"]}
        if what == "get_param_names":
            return {"names": [NODE + ":" + k.rsplit("/", 1)[1] for k in sorted(r.params)]}
        if what in ("get_param", "has_param", "set_param", "delete_param"):
            name = self._param_name(a.get("name"))
            exists = name in r.params
            if what == "has_param":
                return {"exists": exists}
            if what == "get_param":
                if exists:
                    return {"value": json.dumps(r.get_param(name)), "successful": True, "reason": ""}
                return {"value": a.get("default_value", "") or "", "successful": False,
                        "reason": f"Parameter {a.get('name')} does not exist"}
            if what == "delete_param":
                return {"successful": False, "reason": f"{a.get('name')}: parameters of {NODE} cannot be deleted"}
            value = a.get("value")
            try:
                parsed = json.loads(value) if isinstance(value, str) else value
            except ValueError:
                parsed = value
            if not self.link.up:
                raise ValueError("robot unreachable")
            try:
                r.set_param(name, parsed)
            except ToolError as e:
                return {"successful": False, "reason": str(e)}
            return {"successful": True, "reason": ""}
        if what == "message_details":
            d = MESSAGE_DETAILS.get(_norm(a.get("type")))
            if d is None:
                raise ValueError(f"message_details for {a.get('type')!r} is not emulated")
            return {"typedefs": d}
        raise ValueError(f"Service {ROSAPI}{what} does not exist (not emulated)")

    # ------------------------------------------------------------------ the robot's publishers, timers, the link
    def _pump_loop(self):
        while not self._closed.is_set():
            try:
                self._pump()
            except Exception as e:  # keep pumping; record what went wrong
                self._log_error(0, "pump", repr(e))
            self._closed.wait(self.PUMP_S)

    def _pump(self):
        now = self._now()
        if not self.link.up:
            with self.lock:
                conns = list(self.conns.values())
            for c in conns:
                c.shut(1001)
            return
        with self.lock:
            for topic in [t for t, p in self.publishers.items() if not p["clients"] and p["expires"] is not None
                          and now >= p["expires"]]:
                del self.publishers[topic]
            conns = list(self.conns.values())
        if self._next_odom is None:
            self._next_odom = self._next_status = now
        due = []
        if now >= self._next_odom:
            due.append("/odom")
            self._next_odom = max(self._next_odom + 1.0 / self.ODOM_HZ, now)
        status_due = now >= self._next_status
        if status_due:
            self._next_status = max(self._next_status + 1.0 / self.STATUS_HZ, now)
        cache = {}
        for c in conns:
            for topic, s in list(c.subs.items()):
                send = topic in due or (topic == "/rover/status" and (status_due or s.get("latched_pending")))
                if not send or now - s["last"] < s["throttle_s"]:
                    continue
                if topic not in cache:
                    cache[topic] = self.rover.read(topic)
                s["last"], s["latched_pending"] = now, False
                c.send_json({"op": "publish", "topic": topic, "msg": cache[topic]})


# ---------------------------------------------------------------------------------------------- a tiny client
class WebSocketClient:
    """A minimal rosbridge client (for the tests and for checking the emulator by hand)."""

    def __init__(self, url: str, timeout: float = 5.0):
        assert url.startswith("ws://")
        hostport = url[5:].split("/", 1)[0]
        host, port = hostport.rsplit(":", 1)
        self.sock = socket.create_connection((host, int(port)), timeout=timeout)
        try:
            key = base64.b64encode(os.urandom(16)).decode()
            self.sock.sendall((f"GET / HTTP/1.1\r\nHost: {hostport}\r\nUpgrade: websocket\r\n"
                               f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
                               "Sec-WebSocket-Version: 13\r\n\r\n").encode())
            data = b""
            while b"\r\n\r\n" not in data:
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise ConnectionError("handshake failed")
                data += chunk
            if b" 101 " not in data.split(b"\r\n", 1)[0] or accept_key(key).encode() not in data:
                raise ConnectionError("handshake refused")
        except BaseException:
            self.sock.close()
            raise
        self.closed = False

    def send(self, obj):
        self.sock.sendall(frame(0x1, json.dumps(obj).encode(), mask=True))

    def recv(self, timeout: float = 1.0):
        """Next JSON message, or None on timeout. Raises ConnectionError if the server closed the connection."""
        self.sock.settimeout(timeout)
        deadline = time.monotonic() + timeout
        while True:
            try:
                fin, op, data = read_frame(self.sock)
            except socket.timeout:
                return None
            if op == 0x8:
                self.closed = True
                raise ConnectionError("closed by the server")
            if op == 0x1:
                return json.loads(data.decode())
            if time.monotonic() > deadline:
                return None

    def close(self, wait_s: float = 1.0) -> float:
        """Send a close frame and wait for the server's; -> seconds it took."""
        t0 = time.monotonic()
        try:
            self.sock.sendall(frame(0x8, struct.pack("!H", 1000), mask=True))
            self.sock.settimeout(wait_s)
            while True:
                _, op, _ = read_frame(self.sock)
                if op == 0x8:
                    break
        except (OSError, ConnectionError):
            pass
        finally:
            self.sock.close()
            self.closed = True
        return time.monotonic() - t0
