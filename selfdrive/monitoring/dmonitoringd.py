#!/usr/bin/env python3
import gc
from pathlib import Path

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import set_realtime_priority
from openpilot.selfdrive.monitoring.helpers import DriverMonitoring

DISABLE_DM_PATH = Path("/data/disable_driver_monitoring")


def dmonitoringd_thread():
  gc.disable()
  set_realtime_priority(2)

  params = Params()
  pm = messaging.PubMaster(['driverMonitoringState'])
  sm = messaging.SubMaster(['driverStateV2', 'liveCalibration', 'carState', 'controlsState', 'modelV2', 'carControl'], poll='driverStateV2')

  DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"))

  # FrogPilot variables
  disable_driver_monitoring = DISABLE_DM_PATH.is_file()
  driver_view_enabled = params.get_bool("IsDriverViewEnabled")

  # 20Hz <- dmonitoringmodeld
  while True:
    sm.update()
    if not sm.updated['driverStateV2']:
      continue

    # Reload live toggle every ~2 seconds (40 frames @ 20Hz)
    if sm['driverStateV2'].frameId % 40 == 1:
      DM.always_on = params.get_bool("AlwaysOnDM")
      disable_driver_monitoring = DISABLE_DM_PATH.is_file()

    if disable_driver_monitoring:
      # Publish a neutral/attentive state — no distraction tracking, no alerts
      dat = messaging.new_message('driverMonitoringState')
      dat.driverMonitoringState.events = []
      dat.driverMonitoringState.faceDetected = False
      dat.driverMonitoringState.isRHD = DM.wheel_on_right
      dat.driverMonitoringState.awarenessStatus = 1.0
      pm.send('driverMonitoringState', dat)
      continue

    valid = sm.all_checks()
    if valid:
      DM.run_step(sm)
    elif driver_view_enabled:
      DM.face_detected = sm['driverStateV2'].leftDriverData.faceProb > DM.settings._FACE_THRESHOLD or sm['driverStateV2'].rightDriverData.faceProb > DM.settings._FACE_THRESHOLD

    # publish
    dat = DM.get_state_packet(valid=valid or driver_view_enabled)
    pm.send('driverMonitoringState', dat)

    # save rhd virtual toggle every 5 mins
    if (sm['driverStateV2'].frameId % 6000 == 0 and
     DM.wheelpos_learner.filtered_stat.n > DM.settings._WHEELPOS_FILTER_MIN_COUNT and
     DM.wheel_on_right == (DM.wheelpos_learner.filtered_stat.M > DM.settings._WHEELPOS_THRESHOLD)):
      params.put_bool_nonblocking("IsRhdDetected", DM.wheel_on_right)

def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
