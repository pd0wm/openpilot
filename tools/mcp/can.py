"""CAN parsing. Reads can/sendcan packets from a log and decodes them with a DBC."""
import glob
import os

from opendbc import DBC_PATH, get_generated_dbcs
from opendbc.can.dbc import DBC
from opendbc.can.parser import CANDefine, get_raw_value

from openpilot.tools.mcp.common import DEFAULT_MAX_MESSAGES, make_log_reader, rel_time


def list_dbcs() -> dict:
  """List the DBC names available in opendbc."""
  names = {os.path.basename(p)[:-4] for p in glob.glob(os.path.join(DBC_PATH, "*.dbc"))}
  names |= set(get_generated_dbcs())
  return {"count": len(names), "dbcs": sorted(names)}


def _resolve_msg(dbc: DBC, key: str):
  k = str(key).strip()
  if k.lower().startswith("0x") or k.isdigit():
    return dbc.addr_to_msg.get(int(k, 0))
  return dbc.name_to_msg.get(k)


def dbc_info(dbc: str, message: str | None = None) -> dict:
  """Introspect a DBC.

  Without message: list every message (name, address, size, signal count).
  With a message (name or address): list its signals (bit layout, scaling, type) and any value tables.
  """
  d = DBC(dbc)

  if message is None:
    msgs = [
      {"name": m.name, "address": m.address, "address_hex": hex(m.address), "size": m.size, "num_signals": len(m.sigs)}
      for m in sorted(d.msgs.values(), key=lambda m: m.address)
    ]
    return {"dbc": d.name, "count": len(msgs), "messages": msgs}

  msg = _resolve_msg(d, message)
  if msg is None:
    raise ValueError(f"message {message!r} not found in DBC {dbc!r}")

  define = CANDefine(dbc)
  value_tables = define.dv.get(msg.address, {})
  signals = []
  for sig in msg.sigs.values():
    signals.append({
      "name": sig.name,
      "start_bit": sig.start_bit,
      "size": sig.size,
      "is_signed": sig.is_signed,
      "is_little_endian": sig.is_little_endian,
      "factor": sig.factor,
      "offset": sig.offset,
      "type": sig.type,
      "values": value_tables.get(sig.name),
    })
  return {"dbc": d.name, "name": msg.name, "address": msg.address, "address_hex": hex(msg.address), "size": msg.size, "signals": signals}


def _decode_signal(dat: bytes, sig) -> float:
  raw = get_raw_value(dat, sig)
  if sig.is_signed:
    raw -= ((raw >> (sig.size - 1)) & 0x1) * (1 << sig.size)
  return raw * sig.factor + sig.offset


def parse_can(
  route: str,
  dbc: str,
  messages: dict[str, list[str]],
  bus: int = 0,
  source: str = "can",
  start_time: float = 0.0,
  end_time: float | None = None,
  decimation: int = 1,
  include_raw: bool = True,
  mode: str = "rlog",
  max_messages: int = DEFAULT_MAX_MESSAGES,
) -> dict:
  """Decode CAN frames from a route/segment with a DBC and return them as JSON.

  route: a route or segment, e.g. 'a2c.../00000004--569e454120/0', a url, or a local rlog path.
  dbc: a DBC name from list_dbcs(), e.g. 'toyota_nodsu_pt_generated'. Use dbc_info() to find message/signal names.
  messages: maps each message (name or address, e.g. '0x1d2') to the signals to decode.
            Use [] or ["*"] for all signals in the message.
  bus: which CAN bus to read (matches the frame src).
  source: 'can' (received) or 'sendcan' (sent by openpilot).
  start_time / end_time: seconds relative to the start of the route.
  decimation: keep every Nth matching frame per message.
  include_raw: include the raw frame bytes as hex (useful for checking checksums/counters).
  max_messages: hard cap on total rows returned.
  """
  if source not in ("can", "sendcan"):
    raise ValueError("source must be 'can' or 'sendcan'")
  if not messages:
    raise ValueError("specify at least one message, e.g. {'STEERING_LKA': ['STEER_TORQUE_CMD']}")
  if decimation < 1:
    raise ValueError("decimation must be >= 1")

  d = DBC(dbc)

  # resolve requested messages to (name, signals to decode) keyed by address
  by_addr: dict[int, tuple[str, list]] = {}
  for key, sig_names in messages.items():
    msg = _resolve_msg(d, key)
    if msg is None:
      raise ValueError(f"message {key!r} not found in DBC {dbc!r}")
    if not sig_names or list(sig_names) == ["*"]:
      sigs = list(msg.sigs.values())
    else:
      missing = [s for s in sig_names if s not in msg.sigs]
      if missing:
        raise ValueError(f"signals {missing} not found in message {msg.name!r}")
      sigs = [msg.sigs[s] for s in sig_names]
    by_addr[msg.address] = (msg.name, sigs)

  out: dict[str, list] = {}
  seen: dict[int, int] = {}
  for addr, (name, _) in by_addr.items():
    out[name] = []
    seen[addr] = 0

  lr = make_log_reader(route, mode)

  t0 = None
  total = 0
  truncated = False
  for msg in lr:
    if t0 is None:
      t0 = msg.logMonoTime
    if msg.which() != source:
      continue
    t = rel_time(msg.logMonoTime, t0)
    if t < start_time:
      continue
    if end_time is not None and t > end_time:
      break

    for frame in getattr(msg, source):
      if frame.src != bus or frame.address not in by_addr:
        continue
      name, sigs = by_addr[frame.address]

      idx = seen[frame.address]
      seen[frame.address] += 1
      if idx % decimation != 0:
        continue

      dat = bytes(frame.dat)
      row = {"t": round(t, 6), "logMonoTime": msg.logMonoTime, "address": frame.address, "bus": frame.src}
      if include_raw:
        row["raw"] = dat.hex()
      for sig in sigs:
        row[sig.name] = _decode_signal(dat, sig)
      out[name].append(row)

      total += 1
      if total >= max_messages:
        truncated = True
        break
    if truncated:
      break

  return {
    "route": route,
    "dbc": d.name,
    "bus": bus,
    "source": source,
    "t0_mono": t0,
    "counts": {k: len(v) for k, v in out.items()},
    "truncated": truncated,
    "messages": out,
  }
