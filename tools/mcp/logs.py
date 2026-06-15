"""Log viewing as JSON. Wraps openpilot.tools.lib.logreader."""
from openpilot.tools.mcp.common import DEFAULT_MAX_MESSAGES, make_log_reader, get_field, to_jsonable, rel_time


def read_log(
  route: str,
  messages: dict[str, list[str]],
  start_time: float = 0.0,
  end_time: float | None = None,
  decimation: int = 1,
  mode: str = "rlog",
  max_messages: int = DEFAULT_MAX_MESSAGES,
) -> dict:
  """Read selected messages from a route/segment and return them as JSON.

  To keep responses small, you MUST specify exactly which messages and fields you want.

  route: a route or segment, e.g. 'a2c.../00000004--569e454120/0', a connect/useradmin url, or a local rlog path.
  messages: maps each message type to a list of dotted field paths, e.g.
            {"carState": ["vEgo", "gearShifter", "wheelSpeeds.fl"], "liveLocationKalman": ["status"]}.
            Use ["*"] to dump the whole message (can be large).
  start_time / end_time: seconds relative to the start of the route (end_time None = until the end).
  decimation: keep every Nth matching message per type (1 = keep all).
  mode: 'rlog' (full), 'qlog' (decimated on-device log), or 'auto'.
  max_messages: hard cap on total rows returned across all types.
  """
  if not messages:
    raise ValueError("specify at least one message type and its fields, e.g. {'carState': ['vEgo']}")
  if decimation < 1:
    raise ValueError("decimation must be >= 1")

  wanted = {k: list(v) for k, v in messages.items()}
  out: dict[str, list] = {}
  seen: dict[str, int] = {}
  for k in wanted:
    out[k] = []
    seen[k] = 0

  lr = make_log_reader(route, mode)

  t0 = None
  total = 0
  truncated = False
  for msg in lr:
    if t0 is None:
      t0 = msg.logMonoTime
    t = rel_time(msg.logMonoTime, t0)
    if t < start_time:
      continue
    if end_time is not None and t > end_time:
      break

    which = msg.which()
    if which not in wanted:
      continue

    # per-type decimation
    idx = seen[which]
    seen[which] += 1
    if idx % decimation != 0:
      continue

    sub = getattr(msg, which)
    row = {"t": round(t, 6), "logMonoTime": msg.logMonoTime}
    for path in wanted[which]:
      if path == "*":
        whole = to_jsonable(sub)
        row.update(whole if isinstance(whole, dict) else {"value": whole})
      else:
        try:
          row[path] = to_jsonable(get_field(sub, path))
        except (AttributeError, IndexError, KeyError) as e:
          row[path] = f"<error: {e}>"
    out[which].append(row)

    total += 1
    if total >= max_messages:
      truncated = True
      break

  return {
    "route": route,
    "t0_mono": t0,
    "counts": {k: len(v) for k, v in out.items()},
    "truncated": truncated,
    "messages": out,
  }
