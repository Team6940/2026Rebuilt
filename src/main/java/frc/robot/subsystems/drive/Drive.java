// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PoseEstimatorConstants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static Drive instance;

  public static Drive getInstance() {
    return instance;
  }

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 2.255;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
          .withCustomModuleTranslations(getModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getFalcon500(1),
                  TunerConstants.FrontLeft.DriveMotorGearRatio,
                  TunerConstants.FrontLeft.SteerMotorGearRatio,
                  Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                  Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                  Meters.of(TunerConstants.FrontLeft.WheelRadius),
                  KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                  WHEEL_COF));

  // Simulation helper: you can generate simulation-friendly module constants
  // using PhoenixUtil.regulateModuleConstantForSimulation(TunerConstants.FrontLeft) etc.
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final Field2d field2d = new Field2d();

  // Limelight snapshot NT entries — writing 1 triggers a single image save to
  // the Limelight's internal storage, which is viewable post-match at
  // http://<limelight-ip>:5801 under the Snapshots tab.
  private final NetworkTableEntry llLeftSnapshot =
      NetworkTableInstance.getDefault().getTable(RobotContainer.limelightLeft).getEntry("snapshot");
  private final NetworkTableEntry llRightSnapshot =
      NetworkTableInstance.getDefault()
          .getTable(RobotContainer.limelightRight)
          .getEntry("snapshot");

  // PID Controllers for autoMoveToPose
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;
  private final ProfiledPIDController fieldCentricAngleController;

  private void initializeAutoMoveToPoseControllers() {
    // Initialize X and Y PID controllers
    xController.setPID(DriveConstants.MOVE_TO_X_KP, 0.0, DriveConstants.MOVE_TO_X_KD);
    yController.setPID(DriveConstants.MOVE_TO_Y_KP, 0.0, DriveConstants.MOVE_TO_Y_KD);

    // Initialize theta ProfiledPID controller
    thetaController.setPID(DriveConstants.MOVE_TO_THETA_KP, 0.0, DriveConstants.MOVE_TO_THETA_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(
            DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));

    // Set tolerances
    xController.setTolerance(DriveConstants.MOVE_TO_POSITION_TOLERANCE_METERS);
    yController.setTolerance(DriveConstants.MOVE_TO_POSITION_TOLERANCE_METERS);
    thetaController.setTolerance(
        Units.degreesToRadians(DriveConstants.MOVE_TO_ANGLE_TOLERANCE_DEGREES));
  }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    instance = this;
    SmartDashboard.putData("Field", field2d);
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Initialize PID controllers for autoMoveToPose
    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);
    thetaController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    initializeAutoMoveToPoseControllers();

    // Initialize field centric angle controller
    fieldCentricAngleController =
        new ProfiledPIDController(
            Constants.DriveConstants.ANGLE_KP,
            0.0,
            Constants.DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                Constants.DriveConstants.ANGLE_MAX_VELOCITY,
                Constants.DriveConstants.ANGLE_MAX_ACCELERATION));
    fieldCentricAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
                DriveConstants.PP_TRANSLATION_KP, 0.0, DriveConstants.PP_TRANSLATION_KD),
            new PIDConstants(DriveConstants.PP_ROTATION_KP, 0.0, DriveConstants.PP_ROTATION_KD)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // Logging data
    processLog();
    updateOdometry();
  }

  public void processLog() {
    // Reset snapshot triggers so each tag detection only saves one image
    llLeftSnapshot.setNumber(0);
    llRightSnapshot.setNumber(0);
    field2d.setRobotPose(getPose());
    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("Drive/ChassisSpeeds", getChassisSpeeds());
    Logger.recordOutput("Drive/HubRelativeChassisSpeeds", getHubRelativeChassisSpeeds());
    Logger.recordOutput("Drive/DistanceToAllianceHub", getDistanceToAllianceHub());
    Logger.recordOutput("Drive/AllianceHubCenter", getAllianceHubCenter());
    Logger.recordOutput("Drive/RotationToAllianceHub", getRotationToAllianceHub());
  }

  public void updateOdometry() {
    LimelightHelpers.PoseEstimate left = getAcceptedLimelightEstimate(RobotContainer.limelightLeft);
    LimelightHelpers.PoseEstimate right =
        getAcceptedLimelightEstimate(RobotContainer.limelightRight);

    LimelightHelpers.PoseEstimate chosen = null;
    if (left != null && right != null) {
      chosen = left.avgTagDist <= right.avgTagDist ? left : right;
    } else if (left != null) {
      chosen = left;
    } else if (right != null) {
      chosen = right;
    }

    if (chosen != null) {
      double stdDev = PoseEstimatorConstants.tAtoDev.get(chosen.avgTagArea);
      addVisionMeasurement(
          chosen.pose, chosen.timestampSeconds, VecBuilder.fill(stdDev, stdDev, 100000000));
      // according to docs, 6328 template needs no fpgaToCurrentTime
      // conversion
    }
  }

  /**
   * Sends gyro orientation to a single Limelight and fuses its MegaTag2 pose estimate into the pose
   * estimator if the estimate passes all quality gates.
   */
  private LimelightHelpers.PoseEstimate getAcceptedLimelightEstimate(String limelightName) {
    LimelightHelpers.SetRobotOrientation(
        limelightName,
        getPose().getRotation().getDegrees(),
        0,
        gyroInputs.pitchPosition.getDegrees(),
        0,
        gyroInputs.rollPosition.getDegrees(),
        0);

    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    if (mt2 == null) {
      DriverStation.reportWarning(limelightName + " Disconnected!", false);
      Logger.recordOutput("Vision/" + limelightName + "/Connected", false);
      Logger.recordOutput("Vision/" + limelightName + "/TagCount", 0);
      Logger.recordOutput("Vision/" + limelightName + "/AvgTagDist", 0.0);
      Logger.recordOutput("Vision/" + limelightName + "/AvgTagArea", 0.0);
      Logger.recordOutput("Vision/" + limelightName + "/RawPose", new Pose2d());
      Logger.recordOutput("Vision/" + limelightName + "/AcceptedPose", new Pose2d[] {});
      Logger.recordOutput("Vision/" + limelightName + "/RejectedPose", new Pose2d[] {});
      Logger.recordOutput("Vision/" + limelightName + "/GateOmegaOk", false);
      Logger.recordOutput("Vision/" + limelightName + "/GateHasTag", false);
      Logger.recordOutput("Vision/" + limelightName + "/GateDistOk", false);
      Logger.recordOutput("Vision/" + limelightName + "/GateVelOk", false);
      Logger.recordOutput("Vision/" + limelightName + "/Accepted", false);
      return null;
    }

    // Raw measurement data — always logged regardless of acceptance
    Logger.recordOutput("Vision/" + limelightName + "/Connected", true);
    Logger.recordOutput("Vision/" + limelightName + "/TagCount", mt2.tagCount);
    Logger.recordOutput("Vision/" + limelightName + "/AvgTagDist", mt2.avgTagDist);
    Logger.recordOutput("Vision/" + limelightName + "/AvgTagArea", mt2.avgTagArea);
    Logger.recordOutput("Vision/" + limelightName + "/RawPose", mt2.pose);

    // Trigger a snapshot save on the Limelight whenever a tag is visible.
    // Images are saved to the Limelight's internal storage and are viewable
    // post-match at http://<limelight-ip>:5801 under the Snapshots tab.
    // The entry is reset back to 0 in processLog() each loop so only one
    // image is saved per rising edge rather than flooding storage.
    if (mt2.tagCount > 0) {
      NetworkTableEntry snapshotEntry =
          limelightName.equals(RobotContainer.limelightLeft) ? llLeftSnapshot : llRightSnapshot;
      snapshotEntry.setNumber(1);
    }

    // Individual acceptance gate results — use in AdvantageScope to diagnose
    // which gate killed a measurement at any given timestamp
    ChassisSpeeds speeds = getChassisSpeeds();
    boolean omegaOk = Math.abs(speeds.omegaRadiansPerSecond) <= 4 * Math.PI;
    boolean hasTag = mt2.tagCount > 0;
    boolean distOk = mt2.avgTagDist < 3.0;
    boolean velOk = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 2.0;
    boolean accepted = omegaOk && hasTag && distOk && velOk;

    Logger.recordOutput("Vision/" + limelightName + "/GateOmegaOk", omegaOk);
    Logger.recordOutput("Vision/" + limelightName + "/GateHasTag", hasTag);
    Logger.recordOutput("Vision/" + limelightName + "/GateDistOk", distOk);
    Logger.recordOutput("Vision/" + limelightName + "/GateVelOk", velOk);
    Logger.recordOutput("Vision/" + limelightName + "/Accepted", accepted);

    // Log pose into separate accepted/rejected Pose2d arrays so AdvantageScope's
    // 3D view can render accepted poses green and rejected poses red
    if (!accepted) {
      Logger.recordOutput("Vision/" + limelightName + "/AcceptedPose", new Pose2d[] {});
      Logger.recordOutput("Vision/" + limelightName + "/RejectedPose", new Pose2d[] {mt2.pose});
      return null;
    }

    Logger.recordOutput("Vision/" + limelightName + "/AcceptedPose", new Pose2d[] {mt2.pose});
    Logger.recordOutput("Vision/" + limelightName + "/RejectedPose", new Pose2d[] {});
    return mt2;
  }

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), Constants.DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void runFieldRelativeVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds fieldRelativeVelocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getRotation());
    runVelocity(fieldRelativeVelocity);
  }

  public void driveFieldCentric(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply deadband to angular velocity
    double omega =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.DriveConstants.DEADBAND);

    // Square the angular velocity for finer control
    omega = Math.copySign(omega * omega, omega);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * this.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * this.getMaxLinearSpeedMetersPerSec(),
            omega * this.getMaxAngularSpeedRadPerSec());
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    this.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? this.getRotation().plus(new Rotation2d(Math.PI)) : this.getRotation()));
  }

  /**
   * Field-centric drive with custom max speed. Allows you to scale the maximum linear and angular
   * speeds independently.
   *
   * @param xSupplier X-axis joystick input (-1 to 1, forward positive)
   * @param ySupplier Y-axis joystick input (-1 to 1, left positive)
   * @param omegaSupplier Rotational joystick input (-1 to 1, CCW positive)
   * @param maxLinearSpeed Maximum linear speed in meters per second
   * @param maxAngularSpeed Maximum angular speed in radians per second
   */
  public void driveFieldCentricWithMaxSpeed(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double maxLinearSpeed,
      double maxAngularSpeed) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply deadband to angular velocity
    double omega =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.DriveConstants.DEADBAND);

    // Square the angular velocity for finer control
    omega = Math.copySign(omega * omega, omega);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * maxLinearSpeed,
            linearVelocity.getY() * maxLinearSpeed,
            omega * maxAngularSpeed);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    this.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? this.getRotation().plus(new Rotation2d(Math.PI)) : this.getRotation()));
  }

  /**
   * Field relative drive using joystick for linear control and PID for angular control. Possible
   * use cases include snapping to an angle, aiming at a vision target, or controlling absolute
   * rotation with a joystick.
   */
  public void driveFieldCentricAtAngle(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Calculate angular speed
    double omega =
        fieldCentricAngleController.calculate(
            getRotation().getRadians(), rotationSupplier.get().getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the robot's current rotational velocity in radians per second. */
  public double getRobotOmegaRadPerSec() {
    return getChassisSpeeds().omegaRadiansPerSecond;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Returns chassis speeds relative to a target translation, expressed as (radial, tangential).
   * Radial is positive away from the target, tangential is positive counter-clockwise. Uses the
   * turret world position (accounting for its offset from chassis center) as the reference point,
   * and correctly accounts for chassis rotation contributing to the turret pivot's field velocity
   * (v_turret = v_chassis + omega x r_offset).
   */
  public Translation2d getTargetRelativeChassisSpeeds(Translation2d target) {
    Translation2d turretPosition = getTurretWorldPosition();
    Translation2d targetToTurret = turretPosition.minus(target);

    double distance = targetToTurret.getNorm();
    if (distance < 1e-6) {
      return new Translation2d();
    }

    Translation2d radialUnit = targetToTurret.div(distance);
    Translation2d tangentialUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX());

    Translation2d vel = getTurretFieldVelocity();

    double radial = vel.getX() * radialUnit.getX() + vel.getY() * radialUnit.getY();
    double tangential = vel.getX() * tangentialUnit.getX() + vel.getY() * tangentialUnit.getY();

    return new Translation2d(radial, tangential);
  }

  /**
   * Returns the turret pivot's field-relative velocity as a {@link Translation2d} (x = vx, y = vy),
   * correctly accounting for chassis rotation contributing to the turret pivot velocity via the
   * cross-product term {@code omega × r_offset}:
   *
   * <pre>
   *   v_turret = v_chassis + omega × r_offset
   *            = (vx - omega·ry,  vy + omega·rx)
   * </pre>
   *
   * where {@code r_offset} is {@link Constants.TurretConstants#TURRET_OFFSET} rotated into the
   * current field frame. Use this whenever you need field-frame turret velocity (e.g. in {@link
   * frc.robot.util.ProjectileCalculator#solve}).
   */
  public Translation2d getTurretFieldVelocity() {
    ChassisSpeeds robotSpeeds = getChassisSpeeds();
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, getRotation());
    Translation2d rotatedOffset = Constants.TurretConstants.TURRET_OFFSET.rotateBy(getRotation());
    double omega = robotSpeeds.omegaRadiansPerSecond;
    double turretVx = fieldSpeeds.vxMetersPerSecond - omega * rotatedOffset.getY();
    double turretVy = fieldSpeeds.vyMetersPerSecond + omega * rotatedOffset.getX();
    return new Translation2d(turretVx, turretVy);
  }

  /**
   * Returns the turret pivot position in field coordinates, accounting for its offset from the
   * chassis center (defined by {@link Constants.TurretConstants#TURRET_OFFSET} in robot frame). The
   * offset is rotated by the current robot heading before being added to the chassis position.
   */
  public Translation2d getTurretWorldPosition() {
    Pose2d pose = getPose();
    Translation2d rotatedOffset =
        Constants.TurretConstants.TURRET_OFFSET.rotateBy(pose.getRotation());
    return pose.getTranslation().plus(rotatedOffset);
  }

  /** Returns the distance from the robot to a target translation (meters). */
  public double getDistanceToTarget(Translation2d target) {
    return getTurretWorldPosition().getDistance(target);
  }

  /** Returns chassis speeds relative to the hub center, expressed as (radial, tangential). */
  public Translation2d getHubRelativeChassisSpeeds() {
    Translation2d hubCenter = getAllianceHubCenter();
    return getTargetRelativeChassisSpeeds(hubCenter);
  }

  /**
   * Returns the tangential component of the turret's field velocity relative to an arbitrary target
   * (m/s, CCW positive). Positive means the robot is moving in the counter-clockwise direction
   * around the target.
   *
   * <p>Used to compute the turret angular velocity feedforward:
   *
   * <pre>
   *   turretAngularFF (rad/s) = tangentialVelocity / distanceToTarget
   *   turretAngularFF (deg/s) = Math.toDegrees(tangentialVelocity / distanceToTarget)
   * </pre>
   *
   * @param target field-relative target position (e.g. virtual target from the lookahead solver)
   * @return tangential velocity in m/s (CCW positive)
   */
  public double getTurretTangentialVelocityToTarget(Translation2d target) {
    return getTargetRelativeChassisSpeeds(target).getY();
  }

  /** Returns the distance from the robot to the alliance hub center (meters). */
  public double getDistanceToAllianceHub() {
    Translation2d hubCenter = getAllianceHubCenter();
    return getDistanceToTarget(hubCenter);
  }

  /** Returns the field-relative rotation that points the robot toward the alliance hub center. */
  public Rotation2d getRotationToAllianceHub() {
    Translation2d hubCenter = getAllianceHubCenter();
    Translation2d turretPosition = getTurretWorldPosition();
    return hubCenter.minus(turretPosition).getAngle();
  }

  /** Returns chassis speeds relative to the tower center, expressed as (radial, tangential). */
  public Translation2d getTowerRelativeChassisSpeeds() {
    Translation2d towerCenter = getAllianceTowerCenter();
    return getTargetRelativeChassisSpeeds(towerCenter);
  }

  /** Returns the distance from the robot to the alliance tower center (meters). */
  public double getDistanceToAllianceTower() {
    Translation2d towerCenter = getAllianceTowerCenter();
    return getDistanceToTarget(towerCenter);
  }

  /** Returns the field-relative rotation that points the robot toward the alliance tower center. */
  public Rotation2d getRotationToAllianceTower() {
    Translation2d towerCenter = getAllianceTowerCenter();
    Translation2d turretPosition = getTurretWorldPosition();
    return towerCenter.minus(turretPosition).getAngle();
  }

  /*
   * Returns the center point of the alliance hub based on the current alliance.
   */
  public static Translation2d getAllianceHubCenter() {
    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isRedAlliance
        ? Constants.FieldConstants.Hub.oppCenterPoint
        : Constants.FieldConstants.Hub.centerPoint;
  }

  /*
   * Returns the center point of the alliance tower based on the current alliance.
   */
  private static Translation2d getAllianceTowerCenter() {
    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isRedAlliance
        ? Constants.FieldConstants.Tower.oppCenterPoint
        : Constants.FieldConstants.Tower.centerPoint;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  public Command followPPPath(String pathName) {
    return AutoBuilder.followPath(generatePPPath(pathName));
  }

  public PathPlannerPath generatePPPath(String pathName) {
    try {
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      e.printStackTrace();
      return null;
    }
  }

  /**
   * Automatically moves the robot to the target pose using separate PID controllers for X, Y, and
   * theta. This method should be called periodically (e.g., in a command's execute() method) until
   * the robot reaches the target pose.
   *
   * @param targetPose The desired pose to move to (field-relative)
   */
  public void autoMoveToPose(Pose2d targetPose) {
    Pose2d currentPose = getPose();

    double xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());
    double omega =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Clamp linear velocity by vector magnitude
    Translation2d linear = new Translation2d(xVelocity, yVelocity);
    double maxV = getMaxLinearSpeedMetersPerSec();
    if (linear.getNorm() > maxV) {
      linear = linear.times(maxV / linear.getNorm());
    }

    Logger.recordOutput("Drive/AutoMoveToPose/Target", targetPose);
    Logger.recordOutput(
        "AutoMoveToPose/CommandedSpeeds", new ChassisSpeeds(linear.getX(), linear.getY(), omega));

    if (atTargetPose()) {
      stop();
      return;
    }

    runFieldRelativeVelocity(new ChassisSpeeds(linear.getX(), linear.getY(), omega));
  }

  /**
   * Returns whether the robot is at the target pose within specified tolerances.
   *
   * @return true if the robot is within tolerance of the target pose
   */
  public boolean atTargetPose() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal();
  }
}
