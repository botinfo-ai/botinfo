# Policy and binding files

A **policy** says what the owner allows. A **binding** says how those actions reach one connector. Both are TOML.
Policies are loaded strictly: an unknown key, a missing bound or a wrong type is an error. A typo therefore
cannot silently turn a limit off. To check a pair:

```bash
python3 -m reins check --policy policies/simrover.toml --binding bindings/simrover-ros-style.toml
```

## Policy keys

`[policy]`
| Key | Default | Meaning |
|---|---|---|
| `name`, `robot` | required | identifiers shown to the agent and in the audit log |
| `description` | "" | one line for the agent (`reins_policy`) |
| `command_ttl_s` | 5.0 | a pending command (waiting for confirmation), or one with an older `issued_at`, is dropped after this |
| `max_clock_skew_s` | 2.0 | tolerance when judging a caller's `issued_at` |
| `require_request_id` | false | when true, every action call must carry a unique `request_id` |
| `on_busy` | "replace" | a new motion while one runs: `replace` (stop, then start the new one) or `reject` |

`[stop]`
| Key | Default | Meaning |
|---|---|---|
| `latch` | true | after any stop, motion stays refused until the owner resumes |
| `on_client_disconnect` | "stop" | `stop` or `ignore` when the agent's MCP session ends |
| `on_backend_error` | "stop" | `stop` or `ignore` when a call to the robot fails |

`[lease]`
| Key | Default | Meaning |
|---|---|---|
| `keepalive_s` | 0.5 | re-send the running motion this often; must be shorter than the robot's own timeout |
| `max_lease_s` | 5.0 | the longest single motion |
| `renewable` | false | expose `reins_renew` (a heartbeat the agent sends to keep moving) |
| `max_total_s` | 30.0 | the longest stretch of motion, counting renewals and back-to-back commands together |
| `min_rest_s` | 1.0 | how long the robot must rest before a new stretch of motion starts |

`[rate_limits]`: `all = { max, window_s }` counts every call except `reins_stop`. `motion = { max, window_s }`
counts accepted motion and one-shot commands. An action can also carry its own `rate = { max, window_s }`.

`[workspace]` (optional; only for robots whose binding reports a pose): `x = [lo, hi]`, `y = [lo, hi]`,
`margin_m`, `check_every_s`. A motion whose predicted path leaves the box (shrunk by the margin) is refused,
unless it moves back toward the box. While moving, a pose outside the box means stop, and the stop latches.

`[actions.<name>]`
| Key | Meaning |
|---|---|
| `kind` | `motion` (runs on a lease, stoppable), `oneshot` (runs to completion, keeps the robot busy), `instant` (no motion: a light, a face) |
| `description` | shown to the agent |
| `params.<p>` | `{ type = "number" \| "integer" \| "enum" \| "boolean", min, max, values, default, unit, description, confirm_above, confirm_values }` |
| `duration_param` | motion: which number parameter is the lease length in seconds |
| `confirm` | every call needs the owner's confirmation |
| `uninterruptible` | a stop cannot cut it short (for example a Sesame pose); the Sesame binding requires `confirm = true` for these |
| `rate` | this action's own rate limit |

Names: lowercase letters, digits and `_`. Names starting with `reins_` are reserved. Out-of-range values are
refused, never clamped. Any action call may also carry `request_id` (a repeat is refused as a replay) and
`issued_at` (Unix seconds; too old is refused as stale).

## Bindings

- `kind = "sesame-http"`, `url = "http://127.0.0.1:8765"`: the Sesame firmware API. v0 refuses
  `allow_remote = true`.
- `kind = "sesame-mock"`, `patched = true|false`: the firmware mock inside the Reins process.
- `kind = "mcp-tools"`: front an MCP connector. Either `connector = "simrover-ros-style"` (in-process
  simulator) or `command = [...]` (launch an MCP server over stdio; `"{python}"` stands for the Python
  running Reins). Each action maps to a fixed tool call. A string that is exactly `"$param"` is replaced by the
  checked value, and nothing is spliced into a longer string. `[stop]` is required. `[status]` lists the
  only fields the agent will see, each with a type; anything else is redacted. `[pose]` enables the
  workspace check. Full example: `bindings/simrover-ros-style.toml`. Format reference:
  `reins/backends/tooltemplate.py`.
