# Team 6940 — 2026 Robot Code (Rebuilt)

FRC Team 6940's robot code for the **2026 season (Rebuilt)**. Written in Java and built on top of WPILib's command-based framework. The codebase features shoot-on-the-move with an iterative lookahead solver, a velocity-mode turret controller with predictive feedforward, dual-encoder absolute position via the Chinese Remainder Theorem, and full AdvantageKit telemetry/replay support.

> **All collaborators:** please work on your own branch. Submit a Pull Request when you want to merge into `master`.

---

## Setup Instructions

### General

1. Clone this repository.
2. Run `./gradlew` (or `gradlew.bat` on Windows) to download Gradle and all FRC/vendor libraries.
3. Run `./gradlew tasks` to see all available build targets.

### Visual Studio Code (Official IDE)

- Install the [WPILib VS Code extension](https://marketplace.visualstudio.com/items?itemName=wpilibsuite.vscode-wpilib) — requires **Java 17 or later**.
- In `.vscode/settings.json`, set `java.home` to your JDK 17 directory.

### Basic Gradle Commands

| Command | Description |
|---|---|
| `./gradlew deploy` | Build and deploy to the roboRIO |
| `./gradlew build` | Compile the project (add `--info` for details) |
| `./gradlew simulateJavaRelease` | Launch the robot simulation |

### Simulation

- Run `./gradlew simulateJavaRelease` to start simulation.
- Two Xbox controllers are configured (driver on port 0, operator on port 1).
- The simulation uses **maple-sim** (`Arena2026Rebuilt`) for realistic drive physics.
- Open **AdvantageScope** and connect to the simulated NetworkTables to visualize robot state and telemetry in real time.
- AdvantageKit's `REPLAY` mode lets you re-run any match log offline for debugging.

---

## Code Highlights

### Shoot-on-the-Move (`HybridShootCommand`)

The `HybridShootCommand` lets the robot score while driving by compensating for chassis velocity in two ways, selectable via `MotionShotMode`:

**Method A — `DIRECT` (fast 2D map lookup)**
- Decomposes the turret's field-frame velocity into *radial* and *tangential* components relative to the hub.
- Looks up shooter RPS and hood angle from a pre-calibrated 2D interpolation table keyed on `(radialVelocity, distance)`.
- Applies a trigonometric yaw lead: `atan(tangentialVelocity × flightTime / distance)`.
- Simple and deterministic; does not account for where the robot will be when the note lands.

**Method B — `LOOKAHEAD` (iterative virtual-target solver, default)**
- Runs 20 iterations to converge on a *virtual target*: the field position the turret must aim at so the note arrives at the real target after the robot has moved.
  1. Look up flight time for the current effective distance.
  2. Project the turret's future position: `pos + velocity × flightTime`.
  3. Recompute the distance from that projected position to the real target.
- RPS and hood angle are then read from the **static-shot** interpolation table at the converged lookahead distance — no separate radial-velocity map is needed.
- Feeds a **turret angular-velocity feedforward** (deg/s) into the velocity-mode controller so the turret tracks the virtual target smoothly as the robot moves:
  ```
  targetVelFF = toDegrees(tangentialVelocityToVirtualTarget / lookaheadDistance)
  ```
- All intermediate values (virtual target, flight time, FF terms) are logged to AdvantageKit for post-match analysis.

The command also supports a `PASS` mode that lobs the game piece into an open alliance lane, using lane geometry, linear RPS extrapolation by distance, and a simple `atan` lead for lateral motion.

---

### Turret Predictive Velocity Controller (`TurretSubsystem` — `VELOCITY` mode)

The turret operates in three modes (`HYBRID`, `MANUAL`, `VELOCITY`). When shoot-on-the-move is active it enters **VELOCITY** mode, which combines positional correction with two predictive feedforward terms into a single velocity command sent directly to the motor:

```
velocityCmd = kP_position × positionError
            + kFF_targetVel × targetVelFFDegsPerSec   // tracks the moving virtual target
            + kFF_chassis   × (−ωRobot_degs/s)        // counter-rotates with chassis yaw
```

- **Position P-term** — corrects residual pointing error.
- **Target velocity FF** — supplied every loop cycle by `HybridShootCommand` from the lookahead solver so the turret anticipates where the virtual target will be, rather than always chasing where it was.
- **Chassis omega FF** — keeps the turret field-fixed while the drivetrain rotates. Robot CCW positive → turret CW → negative sign applied automatically.
- **Soft deceleration zone** — within 10 ° of either mechanical limit the command is linearly ramped to zero, preventing hard stops.

The combined velocity (deg/s) is clamped and sent to `TurretIO.setVelocity()`, where the motor's internal closed-loop PID receives it as a velocity setpoint with native feedforward support via Phoenix 6's `TorqueCurrentFOC`.

---

### Dual-Encoder Absolute Turret Position (Chinese Remainder Theorem)

The turret uses **two absolute encoders** on different gear ratios to resolve its absolute position across more than one full revolution without a multi-turn encoder. The CRT approach is implemented in `calculateTurretDegsFromEncodersCRT`:

- Encoder 1 has gear ratio `r1 = GEAR_TURRET / GEAR_1 ≈ 3.8` turns per turret revolution.
- Encoder 2 has gear ratio `r2 = GEAR_TURRET / GEAR_2 ≈ 5.11` turns per turret revolution.
- For every plausible complete-rotation count of encoder 1 within the mechanical range, a candidate turret angle is computed: `candidate = (encoder1Deg + k × 360°) / r1`.
- The candidate that best predicts encoder 2's reading (smallest circular error) is selected as the true turret angle.
- This gives unambiguous absolute position at startup with no manual zeroing required.

A slope-based fallback (`calculateTurretDegsFromEncodersSlope`) is also retained for cross-validation logging.

---

### Setpoint Lead Compensator (`SetpointLeadCompensator`)

Hardware mechanisms lag behind rapidly-moving setpoints. `SetpointLeadCompensator` counteracts this *"layback"* effect by:

1. Computing a finite-difference derivative of the setpoint each loop cycle.
2. Running the derivative through a two-stage low-pass filter (configurable α) to suppress noise amplification.
3. Adding `derivative × leadIndex` to the raw setpoint, effectively commanding an over-shot target in the direction of motion.
4. Applying lead only when the smoothed derivative exceeds a configurable threshold, avoiding dither at rest.

The lead index (seconds) is tuned to match the hardware's closed-loop lag. The compensator is used on both the turret and hood subsystems.

---

### Autonomous Routines

Autonomous modes are composed as `SequentialCommandGroup`s using PathPlanner paths combined with `HybridShootCommand` and intake commands. Available routines include:

| Routine | Description |
|---|---|
| `MidOutpost` | Mid start → shoot preloaded → drive to outpost → intake + shoot cycle |
| `MidDepot` | Mid start → shoot → intake from depot → shoot |
| `MidLC` / `MidRC` | Mid start with left / right cycle strategies |
| `LeftDepotCycle` | Left start depot cycling |
| `Left_2Cycles` / `Right_2Cycles` | Two-cycle left / right routines |
| `LeftNA` / `RightNA` | No-auto fallback for left / right starts |
| `RightOutpostCycle` | Right start outpost cycling |

PathPlanner paths are followed via `Drive.followPPPath()`. Alliance flipping is handled automatically using `DriverStation.getAlliance()`.

---

### Vision & Pose Estimation

Two **Limelight** cameras provide AprilTag-based pose corrections fed into the `SwerveDrivePoseEstimator`:

- `limelight-l` (left camera) — supplementary tag coverage.
- `limelight` (right camera) — primary tag detection and pose updates.

Hub AprilTag IDs (2–5, 8–11, 18–21, 24–27) receive higher trust for odometry updates even at large distances. Standard deviation scaling with tag distance and ambiguity is applied via `LimelightHelpers`.

---

## Package Descriptions

### `frc.robot`
Central robot entry point. `Robot.java` manages mode transitions. `RobotContainer.java` declares all subsystems, binds controller buttons, and registers autonomous modes. `Constants.java` holds all tunable numerical constants (motor IDs, PID gains, field geometry, shooter tables, etc.).

### `frc.robot.commands`
All teleop and autonomous commands.

| Class | Purpose |
|---|---|
| `HybridShootCommand` | Shoot-on-the-move with DIRECT or LOOKAHEAD motion compensation |
| `ManualShootFieldRelativeCommand` | Joystick-aimed manual shooting, field-relative |
| `ManualShootCommand` | Robot-relative manual shooting fallback |
| `IntakeCommand` / `IntakeDefaultCommand` | Intake game pieces |
| `ClimbExtendCommand` / `ClimbRetractCommand` | Extend / retract climber |
| `DriveCommands` | Teleop swerve drive (field-relative, robot-relative, auto-aim helpers) |

### `frc.robot.commands.Autos`
Sequential autonomous routines (see table above). Each routine sets the starting pose via PathPlanner and chains path following, shooting, and intaking commands.

### `frc.robot.subsystems.Drive`
Full swerve drive implementation including:
- `Drive.java` — pose estimation, PathPlanner auto builder, vision integration, hub-relative velocity helpers (`getTurretWorldPosition`, `getTurretFieldVelocity`, `getTurretTangentialVelocityToTarget`), field-relative and robot-relative driving.
- `Module.java` — per-module odometry and motor control.
- `ModuleIOTalonFXReal` / `ModuleIOTalonFXSim` — Phoenix 6 hardware and maple-sim implementations.
- `GyroIOPigeon2` / `GyroIONavX` / `GyroIOSim` — gyro abstractions.
- `PhoenixOdometryThread` — high-frequency (250 Hz on CANivore) odometry reading thread.

### `frc.robot.subsystems.Turret`
Multi-mode turret with CRT absolute position, velocity feedforward controller, and setpoint lead compensation. See *Code Highlights* above.

### `frc.robot.subsystems.Shooter`
Flywheel shooter with closed-loop RPS control via Phoenix 6. Supports multiple preset RPS values for different shot types.

### `frc.robot.subsystems.Hood`
Adjustable hood angle (position control) with HYBRID and MANUAL modes, operator nudge input, and the same `SetpointLeadCompensator` pattern as the turret.

### `frc.robot.subsystems.Feeder`
Turntable and feed rollers that stage game pieces and deliver them to the shooter on command.

### `frc.robot.subsystems.Intake`
Ground intake with multiple modes: `INTAKE`, `SHAKE`, `STOPOUT`, `REVERSE`, `OFF`.

### `frc.robot.subsystems.Stretcher`
Stretcher mechanism for game-piece manipulation.

### `frc.robot.subsystems.Climber`
Two-stage climber with extend/retract commands backed by Phoenix 6 motor control.

### `frc.robot.subsystems.Vision`
`LimelightHelpers.java` — full Limelight API wrapper for pose estimation, target acquisition, and pipeline switching.

### `frc.robot.subsystems`
`SuperStructure.java` — singleton façade that selects the correct shoot/intake/climb command based on the active `ControlMode` (`HYBRID` or `MANUAL`) and `ShootMode` (`SCORE` or `PASS`).

### `frc.robot.util`
Utility classes:

| Class | Purpose |
|---|---|
| `ProjectileCalculator` | Shoot-on-the-move solver (Method A direct lookup, Method B iterative lookahead) |
| `Interpolating2DMap` | 2D bilinear interpolation/extrapolation over `(outerKey → (distance → value))` surfaces |
| `SetpointLeadCompensator` | Derivative-based setpoint lead to eliminate hardware layback |
| `TurretVelocityCalculator` | Trapezoidal motion profile + chassis-omega compensation for turret velocity FF |
| `SetpointDerivative` | Finite-difference derivative with low-pass filter |
| `MathUtils` | General math helpers |
| `PhoenixUtil` | Phoenix 6 device configuration helpers |
| `LocalADStarAK` | AdvantageKit-compatible PathPlanner A* pathfinder wrapper |

### `frc.robot.generated`
`TunerConstants.java` — auto-generated swerve module constants (gear ratios, wheel radius, PID gains) from Phoenix Tuner X.

### `frc.robot.Library`
Reusable library code sourced from other FRC teams (credited by package name):

| Package | Source | Contents |
|---|---|---|
| `team3476` | Team 3476 | Timestamped pose tracking, live-editable values, FPGA timer |
| `team1678` | Team 1678 | Unit conversions, CTRE swerve module state helpers |
| `team2910` | Team 2910 | PID controller, rigid-transform / rotation math, interpolating tree map |
| `team6940` | Team 6940 | `MUtils2025`, `TrajectoryPoint` |
| `team1323` | Team 1323 | Moving average filter, HSV→RGB conversion |
| `team95` | Team 95 | Better swerve kinematics and module state |
| `team1706` | Team 1706 | Field-relative speed/acceleration, linear interpolation table |
| `team503` | Team 503 | Utility helpers, interpolating tree map |

---

## Variable Naming Conventions

| Convention | Example | Meaning |
|---|---|---|
| `k[A-Z]...` | `kDriveWheelbaseMeters` | Compile-time constant (found in `Constants.java`) |
| `m_` prefix | `m_instance` | Member (instance) variable |
| `...IO` suffix | `TurretIO`, `GyroIO` | Hardware abstraction interface |
| `...IOInputsAutoLogged` | `TurretIOInputsAutoLogged` | AdvantageKit auto-logged input struct |
| `...Degs` / `...Rps` | `autoSetpointDegs`, `targetRps` | Unit suffix on numeric variables |

---

## Vendor Dependencies

| Library | Version | Purpose |
|---|---|---|
| WPILib GradleRIO | 2026.1.1 | Core FRC Java framework |
| Phoenix 6 | 26.1.1 | CTRE TalonFX / CANcoder / Pigeon 2 motor control |
| PathPlanner | 2026.1.2 | Autonomous path following & PathPlanner GUI integration |
| ChoreoLib | 2026 | Choreo trajectory support |
| AdvantageKit | (vendordep) | Structured logging, simulation replay, auto-logged IOs |
| maple-sim | (vendordep) | High-fidelity swerve drive physics simulation |
| URCL | (vendordep) | Universal Robot Characterization Logging |
| Grapple FRC | 2026 | Additional utility library |
| Studica | (vendordep) | NavX gyro support |

---

## Licenses

- Team 6940 code: see repository license.
- WPILib: BSD — see `WPILib-License.md`.
- PathPlanner: MIT.
- AdvantageKit / AdvantageScope: GNU GPL v3 (portions from FRC 6328 Mechanical Advantage).
- Vendor libraries: see respective vendor licenses.
