# Team 6940 — 2026 Robot Code (Rebuilt)

FRC Team 6940's robot code for the 2026 Robot, **Orion**.

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

The `HybridShootCommand` lets the robot score while driving by automatically compensating for the robot's movement. There are two selectable compensation methods:

**Method A — `DIRECT`**
Adjusts the shooter speed and turret angle based on how fast and in which direction the robot is currently moving relative to the target. It's quick and simple, but only looks at where the robot is right now rather than where it will be when the game piece lands.

**Method B — `LOOKAHEAD` (default)**
Before firing, the code figures out where the robot will be by the time the game piece reaches the target. The turret then aims at that future position rather than the current one, so the game piece arrives on target even though the robot has moved. Shooter speed and hood angle are then chosen based on that adjusted distance. All the calculation results are logged so drivers and programmers can review them after a match.

The command also supports a `PASS` mode that lobs the game piece to a partner robot in an open lane beside the alliance hub, automatically picking the correct lane based on the robot's position on the field.

---

### Turret Predictive Velocity Controller (`TurretSubsystem` — `VELOCITY` mode)

The turret has three control modes: `HYBRID` (auto-aim with driver trim), `MANUAL` (joystick-aimed), and `VELOCITY` (used during shoot-on-the-move).

In **VELOCITY** mode the turret is commanded by speed rather than a fixed angle target. Three things are combined to decide how fast the turret should spin each loop cycle:

- **Pointing correction** — if the turret is slightly off target it adds a small speed push in the right direction.
- **Target tracking** — the shoot-on-the-move solver tells the turret how fast the virtual aim point is moving, so the turret stays ahead of the target instead of always chasing it.
- **Chassis counter-rotation** — when the whole robot spins, the turret automatically spins the opposite way at the same rate so the aim stays fixed on the field even while the drivetrain rotates.

Near the mechanical travel limits, the speed is gradually reduced to avoid slamming into the hard stops.

---

### Dual-Encoder Absolute Turret Position

The turret can rotate more than one full turn, but a normal absolute encoder only reads angles within a single turn. To know the true position at startup without any manual zeroing, **two encoders** are mounted on different gear ratios. Because each encoder wraps around at a different turret angle, cross-referencing their readings produces a unique solution — the only turret angle that is consistent with both encoders at once. This gives the robot reliable absolute position information the moment it powers on.

A second calculation method is also run in parallel and logged for comparison and cross-checking.

---

### PathPlanner Autonomous Paths

During the autonomous period the robot follows pre-made **paths** created in PathPlanner, a graphical path-planning tool. Each path is a smooth curve across the field with target speeds baked in. The drivetrain has a built-in path-following controller that continuously steers all four swerve modules to track the path, correcting for any deviation using the fused odometry position described above. The robot also automatically mirrors paths from blue-alliance coordinates to red-alliance coordinates, so the same routine works on either side of the field without any code changes.

---

### Autonomous Routines

Autonomous modes are composed as `SequentialCommandGroup`s using PathPlanner paths combined with `HybridShootCommand` and intake commands. Available routines include:

| Routine | Description |
|---|---|
| `MidOutpost` | Mid start → shoot preloaded → drive to outpost → intake + shoot cycle |
| `MidDepot` | Mid start → shoot → intake from depot → shoot |
| `LeftDepotCycle` | Left start depot cycling |
| `RightOutpostCycle` | Right start outpost cycling |

PathPlanner paths are followed via `Drive.followPPPath()`. Alliance flipping is handled automatically using `DriverStation.getAlliance()`.

---

### Vision & Pose Estimation

Two **Limelight** cameras (`limelight-l` left and `limelight` right) detect field AprilTags and feed pose corrections into the swerve drive's `SwerveDrivePoseEstimator`. Hub AprilTag IDs are treated as **higher-value landmarks** — readings from these tags are trusted more even at larger distances, giving the robot a more accurate position fix when the hub is visible. All other tags use standard deviation scaling that grows with tag distance and ambiguity to avoid polluting the estimate with unreliable data.

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
Multi-mode turret with CRT absolute position and velocity feedforward controller. See *Code Highlights* above.

### `frc.robot.subsystems.Shooter`
Flywheel shooter with closed-loop RPS control via Phoenix 6. Supports multiple preset RPS values for different shot types.

### `frc.robot.subsystems.Hood`
Adjustable hood angle (position control) with HYBRID and MANUAL modes and operator nudge input.

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
| `SetpointLeadCompensator` | Derivative-based setpoint lead to eliminate hardware layback *(class exists but is not currently used)* |
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
| ChoreoLib | 2026.0.1 | Choreo trajectory support |
| AdvantageKit | 26.0.0 | Structured logging, simulation replay, auto-logged IOs |
| maple-sim | 0.4.0-beta | High-fidelity swerve drive physics simulation |
| URCL | 2026.0.0 | Universal Robot Characterization Logging |
| Grapple FRC | 2026.0.0 | Additional utility library |
| Studica | 2026.0.0 | NavX gyro support |

---

## Licenses

- Team 6940 code: see repository license.
- WPILib: BSD — see `WPILib-License.md`.
- PathPlanner: MIT.
- AdvantageKit / AdvantageScope: GNU GPL v3 (portions from FRC 6328 Mechanical Advantage).
- Vendor libraries: see respective vendor licenses.
