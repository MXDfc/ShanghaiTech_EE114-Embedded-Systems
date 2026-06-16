# EE114 LabProject - LD14P Smart Cruise Car

PPT-style project report outline: [`PROJECT_REPORT.md`](PROJECT_REPORT.md)

## Project Overview

This project is based on the Lab11/12 WHEELTEC L150 LD14P HAL project. It adds a complete "smart cruise" mode:

- LD14P lidar obstacle detection.
- Autonomous low-speed cruise.
- Sector-based obstacle avoidance.
- Short reverse escape when the front area is too close.
- OLED and APP telemetry for quick debugging.

The original Lab11/12 radar demo code is kept in the project. The new default startup mode is `Smart_Cruise_Mode`.

## Project Path

Recommended Keil project path:

```text
LabProject/L150Smart/MDK-ARM/miniBlance.uvprojx
```

LD14 migration variant:

```text
LabProject/L150Smart_LD14/MDK-ARM/miniBlance.uvprojx
```

Use `L150Smart` for the LD14P lidar car and `L150Smart_LD14` for the LD14 lidar car. The smart cruise behavior is intended to stay the same in both builds. The key porting differences are `CONTROL/Lidar.h` selecting the LD14 parser/data layout and UART5 running at `115200` baud for LD14 instead of `230400` baud for LD14P.

This short ASCII path is recommended because older Keil/uVision builds can be unstable with long paths that contain Chinese characters and parentheses.

The original copied project folder is also kept here:

```text
LabProject/WHEELTEC L150(LD14P) 避障巡线雷达小车HAL库_2026.04.13/MDK-ARM/miniBlance.uvprojx
```

Main added module:

```text
CONTROL/smart_cruise.c
CONTROL/smart_cruise.h
```

## Hardware Assumptions

This project reuses the Lab11/12 hardware configuration:

- MCU: STM32F103RC.
- Car platform: WHEELTEC L150 Ackermann car.
- Lidar: LD14P, using the existing `CONTROL/Lidar.c` parser.
- Motors and encoders: using existing `bsp_motor`, encoder, PID and kinematics modules.
- OLED: using existing `bsp_oled` and `CONTROL/show.c`.
- Bluetooth/serial APP telemetry: using the existing USART/printf path.

No new wiring is introduced by this project. If the car cannot receive lidar data, first verify the Lab11/12 original lidar demo runs correctly.

## Software Design

The control loop runs in the existing TIM5 interrupt callback. In Smart mode:

1. `Smart_Cruise_Task()` reads the latest `Dataprocess[]` lidar circle data.
2. Lidar points are divided into front, front-left, front-right, left, right and rear sectors.
3. A small state machine chooses the motion command.
4. The existing `Move_X` and `Move_Z` commands are passed into the original kinematics, PID and PWM output code.

State machine:

- `SMART_STATE_CRUISE_FORWARD`: move forward at low speed.
- `SMART_STATE_AVOID_SLOW`: slow down and steer toward the more open side.
- `SMART_STATE_AVOID_TURN`: continue steering when the front area is blocked.
- `SMART_STATE_ESCAPE_BACK`: reverse briefly when the obstacle is too close.
- `SMART_STATE_PAUSE_STOP`: stop if lidar data is missing.

Default parameters are in `CONTROL/smart_cruise.c`:

```c
SmartCruiseConfig SmartCruise_Config = {
    0.25f,   /* cruise_speed */
    0.12f,   /* slow_speed */
    -0.14f,  /* back_speed */
    0.38f,   /* max_turn_angle */
    850,     /* warn_distance_mm */
    520,     /* stop_distance_mm */
    300,     /* escape_distance_mm */
    80       /* no_data_timeout_ticks */
};
```

## Build And Flash

1. Open `MDK-ARM/miniBlance.uvprojx` with Keil uVision.
2. Confirm the target is `miniBlance` and the device is `STM32F103RC`.
3. Build the project with `F7`.
4. Connect ST-Link or the serial bootloader tool used in the original course materials.
5. Download the generated firmware to the board.
6. Power the car from battery, then place it on the ground with enough space before enabling motion.

## Operation Guide

After reset, the project enters Smart Cruise mode automatically.

Expected OLED display:

- First line: `Mode:Smart`.
- `S`: current state number.
- `F`: front obstacle distance in mm.
- `N`: nearest lidar point distance in mm.
- `L/R`: front-left and front-right sector distances.
- `C`: obstacle encounter counter.
- `VX/VZ`: current forward speed and steering command.

Expected behavior:

- With no obstacle in front, the car cruises forward slowly.
- When an obstacle is within about 850 mm, the car slows and steers toward the side with more free space.
- When an obstacle is within about 520 mm, the car turns more aggressively.
- When an obstacle is within about 300 mm, the car reverses briefly, then retries avoidance.
- If lidar data is missing for about 400 ms, the car stops.

## Debug Checklist

- If OLED does not show `Mode:Smart`, check `Mode = Smart_Cruise_Mode` in `Src/main.c` and `CONTROL/control.c`.
- If the car does not move, check `Flag_Stop`, battery voltage and motor driver wiring.
- If distance values stay at `0`, check LD14P power, UART wiring and whether `lap_count` is updating.
- If the car turns the wrong way, swap the sign returned by `Smart_Select_Turn()` or adjust Ackermann steering direction in the original motor control code.
- If avoidance is too sensitive, increase `warn_distance_mm` only after verifying lidar units are mm.
- If avoidance is too late, increase `stop_distance_mm` and reduce `cruise_speed`.

## Modified Files

- `Src/main.c`: default startup enters `Smart_Cruise_Mode` and resets smart cruise state.
- `CONTROL/control.h`: adds `Smart_Cruise_Mode` and smart cruise function declarations.
- `CONTROL/control.c`: calls `Smart_Cruise_Task()` in the TIM5 control loop when Smart mode is active.
- `CONTROL/show.c`: adds Smart mode OLED page and APP telemetry values.
- `CONTROL/smart_cruise.c`: new lidar sector processing and avoidance state machine.
- `CONTROL/smart_cruise.h`: new data structures, telemetry and public API.
- `MDK-ARM/miniBlance.uvprojx`: adds `smart_cruise.c` to the Keil project.

## Verification Status

Static checks completed:

- Keil project XML can be parsed successfully.
- `smart_cruise.c` is included in the Keil project file.
- LD14P data type matches the existing `Dataprocess[800]` definition.
- The project has C99 enabled in Keil (`uC99=1`).

Hardware build and burn were not run in this environment because Keil command-line tools were not found in `PATH`. Please run the final compile in Keil on the lab PC before burning.
