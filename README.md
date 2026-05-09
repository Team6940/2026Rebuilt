# Team 6940 — 2026 Robot Code (Rebuilt)

FRC Team 6940's robot code for the 2026 Robot, **Orion**.

---

## Setup

1. Clone this repository.
2. Run `./gradlew` (or `gradlew.bat`) to download Gradle and all vendor libraries.
3. Install the [WPILib VS Code extension](https://marketplace.visualstudio.com/items?itemName=wpilibsuite.vscode-wpilib) (Java 17+) and set `java.home` in `.vscode/settings.json`.

### Gradle Commands

| Command | Description |
|---|---|
| `./gradlew deploy` | Build and deploy to the roboRIO |
| `./gradlew build` | Compile the project |
| `./gradlew simulateJavaRelease` | Run simulation |

### Simulation

- Two Xbox controllers: driver on port 0, operator on port 1.
- Uses **maple-sim** (`Arena2026Rebuilt`) for drive physics.
- Connect **AdvantageScope** to simulated NetworkTables for live telemetry.
- AdvantageKit `REPLAY` mode supports offline log re-runs.

---

## Code Highlights

### Shoot-on-the-Move (`HybridShootCommand`)

Two compensation methods:

- **`DIRECT`** — adjusts shooter speed and turret angle based on current robot velocity relative to the target. Fast, but ignores future robot position.
- **`LOOKAHEAD` (default)** — predicts where the robot will be when the game piece arrives, aims at that future position, and selects speed/hood angle accordingly. Results are logged for review.

A `PASS` mode lobs game pieces to a partner robot, automatically selecting the correct lane from the robot's field position.

---

### Turret Predictive Velocity Controller (`VELOCITY` mode)

Modes: `HYBRID` (auto-aim + driver trim), `MANUAL` (joystick), `VELOCITY` (shoot-on-the-move).

In **VELOCITY** mode, three components are summed each loop:

- **Pointing correction** — small speed nudge when off target.
- **Target tracking** — feedforward from the lookahead solver so the turret leads the moving aim point.
- **Chassis counter-rotation** — cancels drivetrain rotation to keep the aim field-fixed.

Speed tapers near mechanical limits.

---

### Dual-Encoder Absolute Turret Position

Two encoders on different gear ratios cross-reference to uniquely identify the true turret angle at startup, even across multiple rotations — no manual zeroing needed. A second method runs in parallel for cross-checking.

---

### Autonomous

Routines are `SequentialCommandGroup`s chaining PathPlanner paths, `HybridShootCommand`, and intake commands. Alliance mirroring is automatic via `DriverStation.getAlliance()`.

| Routine | Description |
|---|---|
| `MidOutpost` | Mid start → shoot preloaded → outpost → intake + shoot |
| `MidDepot` | Mid start → shoot → depot intake → shoot |
| `LeftDepotCycle` | Left start depot cycling |
| `RightOutpostCycle` | Right start outpost cycling |

---

### Vision & Pose Estimation

Two Limelights (`limelight-l`, `limelight`) detect AprilTags and correct `SwerveDrivePoseEstimator`. **Hub tag IDs receive lower standard deviations** (higher trust) even at distance; all other tags scale trust inversely with distance and ambiguity.

---

## Package Descriptions

| Package | Description |
|---|---|
| `frc.robot` | `Robot.java` mode transitions · `RobotContainer.java` subsystems & button bindings · `Constants.java` all tunable values |
| `frc.robot.commands` | All teleop/auto commands (see table below) |
| `frc.robot.commands.Autos` | Sequential auto routines using PathPlanner + shoot/intake commands |
| `frc.robot.subsystems.Drive` | Swerve drive, pose estimation, PathPlanner integration, vision fusion, 250 Hz odometry thread |
| `frc.robot.subsystems.Turret` | Multi-mode turret — CRT absolute position, velocity FF controller |
| `frc.robot.subsystems.Shooter` | Flywheel closed-loop RPS control (Phoenix 6), multiple presets |
| `frc.robot.subsystems.Hood` | Hood angle position control — HYBRID / MANUAL modes, operator nudge |
| `frc.robot.subsystems.Feeder` | Turntable + feed rollers staging and delivering game pieces |
| `frc.robot.subsystems.Intake` | Ground intake — `INTAKE`, `SHAKE`, `STOPOUT`, `REVERSE`, `OFF` |
| `frc.robot.subsystems.Stretcher` | Game-piece stretcher mechanism |
| `frc.robot.subsystems.Climber` | Two-stage climber, extend/retract (Phoenix 6) |
| `frc.robot.subsystems.Vision` | `LimelightHelpers.java` — Limelight API wrapper |
| `frc.robot.subsystems` | `SuperStructure.java` — selects shoot/intake/climb command from `ControlMode` + `ShootMode` |
| `frc.robot.util` | Utility classes (see table below) |
| `frc.robot.generated` | `TunerConstants.java` — auto-generated swerve constants from Phoenix Tuner X |
| `frc.robot.Library` | Library code from other FRC teams (see table below) |

### Commands

| Class | Purpose |
|---|---|
| `HybridShootCommand` | Shoot-on-the-move (DIRECT / LOOKAHEAD) |
| `ManualShootFieldRelativeCommand` | Field-relative manual shooting |
| `ManualShootCommand` | Robot-relative manual shooting |
| `IntakeCommand` / `IntakeDefaultCommand` | Intake game pieces |
| `ClimbExtendCommand` / `ClimbRetractCommand` | Extend / retract climber |
| `DriveCommands` | Teleop swerve drive |

### Utilities (`frc.robot.util`)

| Class | Purpose |
|---|---|
| `ProjectileCalculator` | Shoot-on-the-move solver (DIRECT + LOOKAHEAD) |
| `Interpolating2DMap` | 2D bilinear interpolation surface |
| `SetpointLeadCompensator` | Derivative-based setpoint lead *(exists but not currently used)* |
| `TurretVelocityCalculator` | Trapezoidal motion profile + chassis-omega FF |
| `SetpointDerivative` | Finite-difference derivative with low-pass filter |
| `MathUtils` | General math helpers |
| `PhoenixUtil` | Phoenix 6 device configuration helpers |
| `LocalADStarAK` | AdvantageKit-compatible PathPlanner A* wrapper |

### Libraries (`frc.robot.Library`)

| Package | Source | Contents |
|---|---|---|
| `team3476` | Team 3476 | Timestamped pose tracking, live-editable values, FPGA timer |
| `team1678` | Team 1678 | Unit conversions, CTRE swerve state helpers |
| `team2910` | Team 2910 | PID, rigid-transform / rotation math, interpolating tree map |
| `team6940` | Team 6940 | `MUtils2025`, `TrajectoryPoint` |
| `team1323` | Team 1323 | Moving average filter, HSV→RGB |
| `team95` | Team 95 | Swerve kinematics and module state |
| `team1706` | Team 1706 | Field-relative speed/accel, linear interpolation table |
| `team503` | Team 503 | Utility helpers, interpolating tree map |

---

## Naming Conventions

| Convention | Example | Meaning |
|---|---|---|
| `k[A-Z]...` | `kDriveWheelbaseMeters` | Compile-time constant (`Constants.java`) |
| `m_` prefix | `m_instance` | Member variable |
| `...IO` suffix | `TurretIO` | Hardware abstraction interface |
| `...IOInputsAutoLogged` | `TurretIOInputsAutoLogged` | AdvantageKit auto-logged input struct |
| `...Degs` / `...Rps` | `autoSetpointDegs` | Unit suffix |

---

## Vendor Dependencies

| Library | Version | Purpose |
|---|---|---|
| WPILib GradleRIO | 2026.1.1 | Core FRC Java framework |
| Phoenix 6 | 26.1.1 | TalonFX / CANcoder / Pigeon 2 |
| PathPlanner | 2026.1.2 | Auto path following |
| ChoreoLib | 2026.0.1 | Choreo trajectory support |
| AdvantageKit | 26.0.0 | Structured logging & replay |
| maple-sim | 0.4.0-beta | Swerve physics simulation |
| URCL | 2026.0.0 | Robot characterization logging |
| Grapple FRC | 2026.0.0 | Utility library |
| Studica | 2026.0.0 | NavX gyro support |

---

## Licenses

- Team 6940 code: see repository license.
- WPILib: BSD — `WPILib-License.md`.
- PathPlanner: MIT.
- AdvantageKit / AdvantageScope: GNU GPL v3 (FRC 6328).
- Vendor libraries: see respective vendor licenses.
