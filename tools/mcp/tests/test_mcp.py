import os
import pytest

from cereal import log
from openpilot.tools.mcp import schema, logs, can


@pytest.fixture
def synthetic_log(tmp_path):
  # STEERING_LKA on the toyota DBC, used by the CAN test
  addr = can.dbc_info("toyota_nodsu_pt_generated", "STEERING_LKA")["address"]

  buf = b""
  for i in range(10):
    ev = log.Event.new_message()
    ev.logMonoTime = 10_000_000_000 + i * 100_000_000  # 0.1s spacing
    cs = ev.init("carState")
    cs.vEgo = float(i)
    cs.gearShifter = "drive"
    cs.wheelSpeeds.fl = float(i) + 0.5
    buf += ev.to_bytes()

    ev2 = log.Event.new_message()
    ev2.logMonoTime = 10_000_000_000 + i * 100_000_000 + 1
    frames = ev2.init("can", 1)
    frames[0].address = addr
    frames[0].src = 0
    frames[0].dat = bytes([i, 0, 0, 0, 0, 0, 0, 0])
    buf += ev2.to_bytes()

  path = os.path.join(tmp_path, "rlog")  # no extension -> uncompressed
  with open(path, "wb") as f:
    f.write(buf)
  return path


class TestSchema:
  def test_list_log_messages(self):
    res = schema.list_log_messages()
    names = {m["name"] for m in res["messages"]}
    assert res["count"] == len(res["messages"])
    assert {"carState", "can", "carParams"} <= names

  def test_describe_struct_and_enum(self):
    res = schema.describe_message("carState", max_depth=1)
    assert res["struct"] == "CarState"
    assert res["fields"]["wheelSpeeds"]["kind"] == "struct"
    assert "fl" in res["fields"]["wheelSpeeds"]["fields"]
    assert "drive" in res["fields"]["gearShifter"]["values"]

  def test_describe_list_message(self):
    # 'can' is List(CanData) - should resolve to the element struct
    res = schema.describe_message("can")
    assert res["struct"] == "CanData"
    assert res["fields"]["address"]["kind"] == "uint32"
    assert res["fields"]["dat"]["kind"] == "data"

  def test_describe_named_struct(self):
    res = schema.describe_message("CarParams", schema="car", max_depth=0)
    assert res["struct"] == "CarParams"
    assert len(res["fields"]) > 0

  def test_unknown_message(self):
    with pytest.raises(ValueError):
      schema.describe_message("notARealMessage")


class TestDBC:
  def test_list_dbcs(self):
    res = can.list_dbcs()
    assert res["count"] == len(res["dbcs"])
    assert "toyota_nodsu_pt_generated" in res["dbcs"]

  def test_dbc_info_messages(self):
    res = can.dbc_info("toyota_nodsu_pt_generated")
    assert any(m["name"] == "STEERING_LKA" for m in res["messages"])

  def test_dbc_info_signals_and_values(self):
    res = can.dbc_info("toyota_nodsu_pt_generated", "GEAR_PACKET")
    assert res["name"] == "GEAR_PACKET"
    assert any(s["values"] for s in res["signals"])

  def test_dbc_info_by_address(self):
    by_name = can.dbc_info("toyota_nodsu_pt_generated", "STEERING_LKA")
    by_addr = can.dbc_info("toyota_nodsu_pt_generated", hex(by_name["address"]))
    assert by_name["name"] == by_addr["name"]


class TestReadLog:
  def test_basic_fields(self, synthetic_log):
    res = logs.read_log(synthetic_log, {"carState": ["vEgo", "gearShifter", "wheelSpeeds.fl"]})
    rows = res["messages"]["carState"]
    assert len(rows) == 10
    assert rows[0]["vEgo"] == 0.0
    assert rows[0]["gearShifter"] == "drive"
    assert rows[3]["wheelSpeeds.fl"] == 3.5
    assert rows[0]["t"] == 0.0

  def test_decimation(self, synthetic_log):
    res = logs.read_log(synthetic_log, {"carState": ["vEgo"]}, decimation=2)
    assert [r["vEgo"] for r in res["messages"]["carState"]] == [0.0, 2.0, 4.0, 6.0, 8.0]

  def test_time_window(self, synthetic_log):
    res = logs.read_log(synthetic_log, {"carState": ["vEgo"]}, start_time=0.3, end_time=0.5)
    assert [r["vEgo"] for r in res["messages"]["carState"]] == [3.0, 4.0, 5.0]

  def test_max_messages(self, synthetic_log):
    res = logs.read_log(synthetic_log, {"carState": ["vEgo"]}, max_messages=3)
    assert res["truncated"]
    assert len(res["messages"]["carState"]) == 3

  def test_bad_field_is_reported(self, synthetic_log):
    res = logs.read_log(synthetic_log, {"carState": ["nope"]})
    assert "error" in res["messages"]["carState"][0]["nope"]

  def test_requires_messages(self, synthetic_log):
    with pytest.raises(ValueError):
      logs.read_log(synthetic_log, {})


class TestParseCAN:
  def test_decode_signals_and_raw(self, synthetic_log):
    res = can.parse_can(synthetic_log, "toyota_nodsu_pt_generated", {"STEERING_LKA": ["*"]}, bus=0)
    rows = res["messages"]["STEERING_LKA"]
    assert len(rows) == 10
    assert "raw" in rows[0] and rows[0]["raw"] == "00" * 8
    assert "STEER_TORQUE_CMD" in rows[0]
    assert rows[0]["address"] == res["messages"]["STEERING_LKA"][0]["address"]

  def test_message_by_address(self, synthetic_log):
    addr = can.dbc_info("toyota_nodsu_pt_generated", "STEERING_LKA")["address"]
    res = can.parse_can(synthetic_log, "toyota_nodsu_pt_generated", {hex(addr): []}, bus=0)
    assert len(res["messages"]["STEERING_LKA"]) == 10

  def test_wrong_bus_returns_nothing(self, synthetic_log):
    res = can.parse_can(synthetic_log, "toyota_nodsu_pt_generated", {"STEERING_LKA": ["*"]}, bus=1)
    assert res["counts"]["STEERING_LKA"] == 0

  def test_unknown_message(self, synthetic_log):
    with pytest.raises(ValueError):
      can.parse_can(synthetic_log, "toyota_nodsu_pt_generated", {"NOT_A_MSG": []})

  def test_unknown_signal(self, synthetic_log):
    with pytest.raises(ValueError):
      can.parse_can(synthetic_log, "toyota_nodsu_pt_generated", {"STEERING_LKA": ["NOPE"]})
