#!/usr/bin/env python
import zmq
from cereal import car, log
from selfdrive.config import Conversions as CV
from selfdrive.services import service_list
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.boardd.boardd import can_capnp_to_can_list
import selfdrive.messaging as messaging
from common.kalman.simple_kalman import KF1D
from common.numpy_fast import clip


# mocked car interface to work with chffrplus
TS = 0.01  # 100Hz
YAW_FR = 0.2 # ~0.8s time constant on yaw rate filter
# low pass gain
LPG = 2 * 3.1415 * YAW_FR * TS / (1 + 2 * 3.1415 * YAW_FR * TS)

GAS_MAX = 255.


class CarInterface(object):
  def __init__(self, CP, sendcan=None):

    self.CP = CP

    cloudlog.debug("Using Mock Car Interface")
    context = zmq.Context()

    # TODO: subscribe to phone sensor
    self.sensor = messaging.sub_sock(context, service_list['sensorEvents'].port, conflate=True)
    self.gps = messaging.sub_sock(context, service_list['gpsLocationExternal'].port, conflate=True)

    self.speed = 0.
    self.prev_speed = 0.
    self.yaw_rate = 0.
    self.yaw_rate_meas = 0.

    self.fix_ok = False
    self.sendcan = sendcan

    self.set_speed = 0
    self.enabled = False
    self.prev_enabled = False

    self.can = messaging.sub_sock(context, service_list['can'].port)

    # vEgo kalman filter
    dt = 0.01
    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=np.matrix([[0.0], [0.0]]),
                         A=np.matrix([[1.0, dt], [0.0, 1.0]]),
                         C=np.matrix([1.0, 0.0]),
                         K=np.matrix([[0.12287673], [0.29666309]]))
    self.v_ego = 0.0

  @staticmethod
  def compute_gb(accel, speed):
    return accel
    #ff = float(0.5 * speed / 40.)
    #return float(accel) / 5.0 + ff

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.0

  @staticmethod
  def get_params(candidate, fingerprint):

    ret = car.CarParams.new_message()

    ret.carName = "mock"
    ret.carFingerprint = candidate

    ret.openpilotLongitudinalControl = True
    ret.safetyModel = car.CarParams.SafetyModels.allOutput

    # FIXME: hardcoding honda civic 2016 touring params so they can be used to
    # scale unknown params for other cars
    ret.mass = 1700.
    ret.rotationalInertia = 2500.
    ret.wheelbase = 2.70
    ret.centerToFront = ret.wheelbase * 0.5
    ret.steerRatio = 13. # reasonable
    ret.longPidDeadzoneBP = [0.]
    ret.longPidDeadzoneV = [0.]
    ret.tireStiffnessFront = 1e6    # very stiff to neglect slip
    ret.tireStiffnessRear = 1e6     # very stiff to neglect slip
    ret.steerRatioRear = 0.


    ret.steerActuatorDelay = 0.

    # limits
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [1.0]
    ret.brakeMaxBP = [0.]
    ret.brakeMaxV = [0.]
    ret.steerMaxBP = [0.]
    ret.steerMaxV = [0.]  # 2/3rd torque allowed above 45 kph

    # longitudinal PID
    ret.longitudinalKpBP = [0., 5., 35.]
    ret.longitudinalKpV = [0.5, 0.5, 0.5]
    ret.longitudinalKiBP = [0., 35.]
    ret.longitudinalKiV = [0.05, 0.05]

    # lateral PID
    ret.steerKiBP, ret.steerKpBP = [[0.], [0.]]
    ret.steerKiV, ret.steerKpV = [[0.], [0.]]

    ret.enableCruise = True
    ret.enableCamera = True

    return ret

  # returns a car.CarState
  def update(self, c):
    # get basic data from phone and gps since these are not on CAN
    sensors = messaging.recv_sock(self.sensor)
    if sensors is not None:
      for sensor in sensors.sensorEvents:
        if sensor.type == 4:  # gyro
          self.yaw_rate_meas = -sensor.gyro.v[0]

    gps = messaging.recv_sock(self.gps)
    if gps is not None:
      self.prev_speed = self.speed # Save prev speed for accel calculation
      self.fix_ok = gps.gpsLocationExternal.flags & 1
      self.speed = gps.gpsLocationExternal.speed

    # Parse can data
    rcv = messaging.drain_sock(self.can)
    for r in rcv:
      for msg in r.can:
        if msg.address == 1:
          self.enabled = bool(ord(msg.dat[0]))
        elif msg.address == 2:
          self.set_speed = ord(msg.dat[0]) / 3.6

    # create message
    ret = car.CarState.new_message()

    # Speed
    v_ego_x = self.v_ego_kf.update(self.speed)
    ret.vEgoRaw = self.speed
    ret.vEgo = float(v_ego_x[0])
    ret.aEgo = float(v_ego_x[1])
    ret.brakePressed = ret.aEgo < -0.5

    ret.cruiseState.speed = self.set_speed
    ret.cruiseState.enabled = self.enabled
    ret.cruiseState.available = True

    self.yawRate = LPG * self.yaw_rate_meas + (1. - LPG) * self.yaw_rate
    ret.yawRate = self.yaw_rate
    ret.standstill = ret.vEgo < 0.1
    ret.wheelSpeeds.fl = self.speed
    ret.wheelSpeeds.fr = self.speed
    ret.wheelSpeeds.rl = self.speed
    ret.wheelSpeeds.rr = self.speed
    curvature = self.yaw_rate / max(self.speed, 1.)
    ret.steeringAngle = curvature * self.CP.steerRatio * self.CP.wheelbase * CV.RAD_TO_DEG

    events = []
    if not self.fix_ok:
     events.append(create_event('gpsFixLost', [ET.NO_ENTRY, ET.SOFT_DISABLE]))

    if self.enabled and not self.prev_enabled:
      events.append(create_event('pcmEnable', [ET.ENABLE]))
    elif not self.enabled:
      events.append(create_event('pcmDisable', [ET.USER_DISABLE]))

    ret.events = events
    self.prev_enabled = self.enabled

    return ret.as_reader()

  def apply(self, c, perception_state=log.Live20Data.new_message()):
    can_sends = []

    if c.enabled:
      gas = int(clip(c.actuators.gas * GAS_MAX, 0, GAS_MAX))
    else:
      gas = 0

    can_sends.append((10, 0, chr(gas), 0))
    self.sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
