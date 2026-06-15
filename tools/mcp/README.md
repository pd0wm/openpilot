# openpilot MCP server

An [MCP](https://modelcontextprotocol.io) server that exposes openpilot's route,
log, schema and CAN tooling so an LLM client (Claude Code, Claude Desktop, etc.)
can inspect drives without you writing a script each time.

It's a thin wrapper around the existing libraries:
`tools/lib/route.py`, `tools/lib/logreader.py`, `cereal/*.capnp`, and `opendbc`.

## Running

The server speaks MCP over stdio:

```bash
uv run python -m openpilot.tools.mcp
```

Register it with an MCP client. For Claude Code, add to `.mcp.json` (or run
`claude mcp add`):

```json
{
  "mcpServers": {
    "openpilot": {
      "command": "uv",
      "args": ["run", "python", "-m", "openpilot.tools.mcp"],
      "cwd": "/path/to/openpilot"
    }
  }
}
```

## Tools

### Routes
- **`route_info(route, data_dir=None)`** — segment count and per-segment file
  availability (rlog/qlog/cameras). Wraps `Route`.

### Schema introspection
- **`list_log_messages()`** — all message types in a log (the members of the
  `Event` union), with a `deprecated` flag.
- **`describe_message(message, schema="log", max_depth=2)`** — fields and types of
  a message or struct, recursively. Works on `Event` members (`carState`, `can`,
  …) and named structs (`CanData`, `CarParams`, …). Enums list their values.

### Log viewing
- **`read_log(route, messages, start_time=0, end_time=None, decimation=1, mode="rlog", max_messages=10000)`**
  — read selected messages as JSON. Wraps `LogReader`.

### CAN
- **`list_dbcs()`** — available DBC names in opendbc.
- **`dbc_info(dbc, message=None)`** — list a DBC's messages, or a message's signals
  (bit layout, scaling, value tables).
- **`parse_can(route, dbc, messages, bus=0, source="can", start_time=0, end_time=None, decimation=1, include_raw=True, mode="rlog", max_messages=10000)`**
  — decode `can`/`sendcan` frames with a DBC. Returns decoded signals plus the raw
  frame bytes as hex (so you can check checksums/counters yourself).

## Avoiding huge responses

Logs are large, so `read_log` and `parse_can` make you say exactly what you want:

- **You must pass `messages`** — a map of message type → fields/signals to extract.
  - `read_log`: dotted field paths, e.g. `{"carState": ["vEgo", "wheelSpeeds.fl"]}`.
    `["*"]` dumps the whole message.
  - `parse_can`: signal names, e.g. `{"STEERING_LKA": ["STEER_TORQUE_CMD"]}`.
    `[]` or `["*"]` decodes all signals. Messages may be named or addresses (`"0x2e4"`).
- **`start_time` / `end_time`** — seconds relative to the start of the route.
- **`decimation`** — keep every Nth matching message (per type).
- **`max_messages`** — hard cap on rows returned; the response sets `truncated: true`
  if it was hit.

`route` can be a route/segment name (`<dongle>/<logid>` or `.../<logid>--<seg>/0`),
a connect/useradmin URL, or a local rlog path.

## Examples

```
# what messages exist, and what's in carState?
list_log_messages()
describe_message("carState")

# speed and steering angle, once per ~2s, from the qlog
read_log("a2c.../0000--569e/0", {"carState": ["vEgo", "steeringAngleDeg"]},
         decimation=200, mode="qlog")

# decode steering CAN over the first 10s
dbc_info("bmw_sp2018", "0x271")
parse_can("a2c.../0000--569e/0", "bmw_sp2018", {"0x271": ["*"]}, bus=0, end_time=10)
```

## Tests

```bash
pytest tools/mcp/tests/test_mcp.py
```
