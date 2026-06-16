# L150Smart_LD14 - LD14 Smart Cruise Variant

This project is the LD14 lidar variant of `LabProject/L150Smart`.

## What Changed

- The smart cruise behavior is kept the same as the LD14P version.
- `CONTROL/Lidar.h` selects `LD14 = 1` and `LD14P = 0`.
- `Src/usart.c` sets the lidar UART5 baud rate to `115200`, matching the original LD14 factory project. The LD14P build uses `230400`.
- Lidar processed data uses the LD14 layout:
  - `PointDataProcessDef Dataprocess[400]`
  - each point contains `distance` and processed `angle`
  - one lap is limited to about 390 valid points
- The LD14P version uses:
  - `LidarPointStructDef Dataprocess[800]`
  - each point structure directly contains `distance`, `confidence`, and `angle`
  - one lap is limited to about 720 valid points

## Recommended Keil Project

```text
LabProject/L150Smart_LD14/MDK-ARM/miniBlance.uvprojx
```

## Expected Behavior

The runtime behavior should match the LD14P smart cruise project:

- low-speed autonomous cruise
- 270-degree sector obstacle analysis
- direction lock to reduce left/right oscillation
- wall-corner exit-straight behavior
- reverse escape when the front distance is too close
- OLED, Bluetooth APP, and Keil Watch telemetry

## Debug Notes

Use the same Watch variables as the LD14P build:

- `Watch_Front_Hit_Angle_X10`
- `Watch_Front_Hit_MM`
- `Watch_Front_Point_Count`
- `Watch_Nearest_Body_Angle_X10`
- `Watch_Front_Offset_X10`
- `Smart_Front_Angle_Offset`

For first bring-up on LD14 hardware, also watch:

- `receive_cnt`: increases when UART5 receives a CRC-valid lidar frame.
- `lap_count`: should settle around the LD14 one-lap point count, usually below 390.
- `Pack_Data.start_angle` and `Pack_Data.end_angle`: should change continuously.

If these values stay at zero, check that UART5 is `115200` baud and that the LD14 lidar TX line is connected to MCU `PD2 / UART5_RX`.

If the car direction appears rotated after switching hardware, tune
`Smart_Front_Angle_Offset` in Keil Watch or in `CONTROL/smart_cruise.c`.

## LD14 vs LD14P Porting Summary

Both lidar projects parse the same basic 47-byte frame style:

```text
0x54, length, speed, start_angle, 12 * (distance_low, distance_high, confidence), end_angle, timestamp, crc
```

The UART parser is almost the same, but the factory projects differ in UART5
baud rate and processed one-lap data buffer type/size. The smart-cruise module
only reads `distance` and `angle`, so it can reuse the same obstacle-avoidance
algorithm after selecting the correct lidar branch in `Lidar.h`.
