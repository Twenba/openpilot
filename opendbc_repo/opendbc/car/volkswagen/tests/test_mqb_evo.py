"""MQB Evo (Skoda Kodiaq Mk2) on the carrot MEB curvature stack: camera-harness CAN simulation."""
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, VolkswagenFlags

DBC = "vw_mqbevo_2024"

# name -> (Hz, bus, values): what the car (bus 0) and the stock camera (bus 2) send
MSGS = {
  "ESC_51": (50, 0, {"VL_Radgeschw": 50, "VR_Radgeschw": 50, "HL_Radgeschw": 50, "HR_Radgeschw": 50}),
  "LWI_01": (100, 0, {"LWI_Lenkradwinkel": 5.0}),
  "LH_EPS_03": (100, 0, {"EPS_Lenkmoment": 20}),
  "QFK_01": (50, 0, {"LatCon_HCA_Status": 2, "Curvature": 0.001}),
  "ESC_50": (50, 0, {}),
  "Motor_51": (50, 0, {"TSK_Status": 3}),
  "Motor_14": (10, 0, {}),
  "Motor_54": (50, 0, {"Engine_On": 1}),
  "GRA_ACC_01": (33, 0, {"GRA_Typ_Hauptschalter": 1}),
  "Gateway_72": (10, 0, {"ZV_02_alt": 1}),
  "ZV_02": (5, 0, {}),
  "Gateway_73": (20, 0, {"GE_Fahrstufe": 8}),  # D
  "Airbag_02": (5, 0, {"AB_Gurtschloss_FA": 3}),
  "ESP_21": (50, 0, {}),
  "Blinkmodi_02": (1, 0, {}),
  "SMLS_01": (1, 0, {}),
  "KLR_01": (16, 0, {}),
  "MEB_ACC_01": (16, 0, {"ACC_Wunschgeschw_02": 80}),
  "LDW_02": (10, 2, {}),
  "TA_01": (10, 2, {}),
  "HCA_03": (50, 2, {}),
  "EA_01": (2, 2, {}),
  "EA_02": (2, 2, {}),
}


def _fingerprint(packer):
  fp = {b: {} for b in range(8)}
  for name, (_, bus, _) in MSGS.items():
    msg = packer.dbc.name_to_msg[name]
    fp[bus][msg.address] = msg.size
  return fp


def test_kodiaq_mk2_platform_flags():
  flags = CAR.SKODA_KODIAQ_MK2.config.flags
  assert flags & VolkswagenFlags.MEB and flags & VolkswagenFlags.MEB_GEN2
  assert flags & VolkswagenFlags.MQB_EVO and flags & VolkswagenFlags.MQB_EVO_GEN2
  assert CAR.SKODA_KODIAQ_MK2.config.dbc_dict["pt"] == DBC


def test_kodiaq_mk2_camera_harness(monkeypatch):
  errors = []
  orig = carlog.error
  monkeypatch.setattr(carlog, "error", lambda msg, *a, **k: (errors.append(str(msg)), orig(msg, *a, **k)))

  car = CANPacker(DBC)
  CP = CarInterface.get_params(CAR.SKODA_KODIAQ_MK2, _fingerprint(car), [], alpha_long=True, is_release=False, docs=False)
  assert CP.networkLocation == structs.CarParams.NetworkLocation.fwdCamera
  assert CP.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.volkswagenMeb
  assert not CP.openpilotLongitudinalControl and CP.pcmCruise and CP.radarUnavailable
  assert CP.flags & VolkswagenFlags.STOCK_EA_PRESENT and CP.flags & VolkswagenFlags.ALT_GEAR

  CI = CarInterface(CP)
  sent = set()
  cs = None
  for frame in range(1000):
    t = int(frame * 0.01 * 1e9)
    frames = []
    for name, (hz, bus, vals) in MSGS.items():
      if frame % max(1, round(100 / hz)) == 0:
        frames.append(car.make_can_msg(name, bus, dict(vals)))
    cs = CI.update([(t, frames)])
    CC = structs.CarControl()
    CC.enabled = CC.latActive = frame > 300
    CC.actuators.curvature = 0.002
    _, can_sends = CI.apply(CC.as_reader(), t)
    sent |= {(addr, bus) for addr, _, bus in can_sends}

  assert cs.canValid and not cs.canTimeout
  assert cs.gearShifter == structs.CarState.GearShifter.drive
  assert cs.cruiseState.enabled and not cs.accFaulted
  assert abs(cs.cruiseState.speed - 80 / 3.6) < 0.1
  assert not cs.steerFaultTemporary and not cs.steerFaultPermanent
  assert (0x303, 0) in sent  # HCA_03 curvature command to the EPS
  assert not [e for e in errors if "unknown signal" in e or "not found" in e]
