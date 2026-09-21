# Minimal Shooter Test Unit Design

Date: 2026-09-21

## Goal

Extract the infantry gimbal's firing mechanism into a minimal test unit that runs on the existing STM32F407 control board. The unit controls only the two friction-wheel motors and the feeder motor while preserving the existing CAN motor stack.

The first version is intended for safe, incremental bench testing. It does not include vision, referee-system heat control, UI, jam recovery, or gimbal/chassis control.

## Scope

### Included

- Existing STM32F407 board and CubeMX peripheral configuration.
- CAN1 motor feedback and current command transport.
- Left friction wheel: M3508, feedback ID `0x201`.
- Right friction wheel: M3508, feedback ID `0x202`.
- Feeder: M2006 with 36:1 reduction, feedback ID `0x203`.
- Existing `cMotor`, PID, sliding-mode controller, and CAN driver code where practical.
- DT7/DR16 as one command source.
- A command interface that can also be populated by a serial command source later.
- Emergency stop and DT7 disconnect protection.
- Friction-wheel spin-up, readiness detection, single shot, and continuous fire.

### Excluded

- Yaw and pitch control.
- Chassis communication and follow control.
- Vision aiming and `suggest_fire`.
- Referee-system data and heat prediction.
- UI and indicator behavior.
- Jam detection and automatic feeder reversal.
- Shot detection based on friction-wheel speed drop.
- Serial receive/parser implementation.

## Architecture

The firing logic will be separated from its command sources and hardware transport:

```text
DT7 adapter ---------+
                     +--> ShooterCommand --> ShooterController --> motor targets/current buffers
Serial adapter ------+                              |
                                                    +--> ShooterStatus
```

`ShooterController` owns firing state and target generation. It does not parse DT7 packets or serial bytes. Command adapters translate their input into the same `ShooterCommand` structure.

The existing CAN interrupt continues to update motor feedback. The shooter task calls the controller at a fixed 1 ms period, calculates controllers, writes the current values into the existing CAN buffer, and sends the enabled CAN frame.

## Command Interface

```cpp
struct ShooterCommand
{
    bool armed;
    bool friction_on;
    bool fire_once;
    bool fire_hold;
    bool emergency_stop;
};
```

- `armed`: permits operation. False always forces the safe state.
- `friction_on`: requests friction-wheel spin-up.
- `fire_once`: an edge-triggered request for one feeder step.
- `fire_hold`: requests repeated feeder steps while held.
- `emergency_stop`: immediately overrides every other command.

The DT7 adapter is responsible for setting `armed` false when its online timeout expires. A future serial adapter can populate the same structure without changing the shooter controller.

## State Machine

```text
SAFE --armed && friction_on--> SPINUP
SPINUP --speed stable---------> READY
READY --fire request----------> FIRING
FIRING --feeder at target-----> COOLDOWN
COOLDOWN --interval elapsed---> READY

Any state --stop/disarm/disconnect--> SAFE
SPINUP/READY --friction off---------> SAFE
```

### SAFE

- Friction-wheel speed targets are zero.
- Feeder output is zero.
- Pending one-shot requests and timing counters are cleared.
- On transition into `SAFE`, the feeder target is synchronized to its current measured position so that re-arming cannot resume an old movement.

### SPINUP

- Left friction-wheel target: `-6200 rpm`.
- Right friction-wheel target: `+6200 rpm`.
- Both wheels must remain inside the configured ready band for 20 consecutive controller updates before entering `READY`.
- Leaving the ready band resets the consecutive-ready counter.

### READY

- Friction wheels remain speed-controlled.
- A rising `fire_once` request or active `fire_hold` request starts one feeder step.
- The new feeder target is based on the previous commanded target, not an arbitrary repeatedly sampled current position.

### FIRING

- One shot advances the feeder target by 45 output-shaft degrees.
- The feeder sliding-mode position controller tracks the target.
- No new shot target can be created while the state is `FIRING`.
- Arrival requires position error within a configured tolerance for consecutive updates, preventing noise from completing a shot early.
- A timeout returns the controller to `SAFE`; automatic reverse recovery is deliberately excluded from version one.

### COOLDOWN

- The feeder holds its completed target.
- After the configured inter-shot interval, the controller returns to `READY`.
- If `fire_hold` remains active, the next update may start another shot.
- A `fire_once` signal held high does not retrigger; it must fall and rise again.

## Motor Control

The existing motor IDs and CAN1 `0x200` command frame remain unchanged:

| Motor | Feedback ID | Command bytes |
|---|---:|---:|
| Left friction wheel | `0x201` | 0-1 |
| Right friction wheel | `0x202` | 2-3 |
| Feeder | `0x203` | 4-5 |

The friction wheels use their existing speed PID. The feeder uses its existing position sliding-mode controller and multi-turn encoder conversion.

The controller separates target/state updates from CAN transmission. Only the task or CAN adapter writes and sends the final current frame.

## Source Layout

New application files:

```text
User/Shooter/ShooterController.hpp
User/Shooter/ShooterController.cpp
User/Shooter/ShooterCommand.hpp
User/Tasks/ShooterTask/ShooterTask.hpp
User/Tasks/ShooterTask/ShooterTask.cpp
```

Existing reusable modules:

```text
User/MotorC.*
User/MCUDriver/CAN/bsp_can.*
User/Framework/PID/pidC.*
User/Framework/Sliding/Sliding.*
User/Framework/RemoteC/RemoteC.*
User/MCUDriver/REMOTEC/remoteio.*
```

The original `Shootc` remains in the copied repository as reference during the first implementation, but it is not called by the minimal shooter task. Gimbal, INS, vision, and referee tasks will not be part of the active minimal runtime.

## Error Handling And Safety

- Startup state is always `SAFE`.
- Emergency stop, disarm, and DT7 timeout have identical priority and behavior.
- State transitions never depend on the fire input remaining active after a shot begins.
- Friction readiness must be sustained, not observed for one sample.
- A feeder movement timeout forces `SAFE` rather than starting another movement.
- Motor current commands are explicitly written as zero in `SAFE`; stale CAN-buffer values are not trusted.
- Bench testing starts without ammunition, with the mechanism restrained and friction wheels guarded.

## Verification

### Host-side state tests

The state-transition core will be testable without STM32 peripherals. Tests cover:

- Startup remains safe with zero targets.
- Spin-up begins only when armed and friction is requested.
- Readiness requires consecutive in-range samples.
- A one-shot edge produces exactly one 45-degree target increment.
- Holding `fire_once` does not retrigger.
- Holding `fire_hold` retriggers only after firing and cooldown finish.
- Releasing a short fire request does not prevent the active shot from completing.
- Stop, disarm, and disconnect force safe outputs from every state.
- Feeder timeout forces `SAFE`.

### Firmware verification

- Configure and build the cross-compiled firmware from a clean build directory.
- Confirm only the required FreeRTOS tasks are active.
- Confirm CAN1 `0x200` buffer positions match IDs `0x201` through `0x203`.
- With motor power disconnected, observe command transitions through debug output or CAN capture.
- With no ammunition, verify friction direction/speed, one 45-degree feeder step, held-fire sequencing, and emergency stop.

## Acceptance Criteria

- The project builds for STM32F407 from a clean configuration.
- On startup or command loss, all three current commands are zero.
- Both friction wheels reach their intended directions and target speeds before firing is enabled.
- One one-shot edge creates one and only one 45-degree feeder movement.
- Continuous fire produces sequential 45-degree movements separated by the configured interval.
- The minimal runtime has no functional dependency on gimbal, chassis, vision, referee, UI, heat-control, or jam-recovery code.
