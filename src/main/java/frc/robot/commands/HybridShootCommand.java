package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure.ShootMode;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.util.ProjectileCalculator;
import frc.robot.util.ProjectileCalculator.ShotSolution;
import frc.robot.util.simulation.TrajectorySimulator;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class HybridShootCommand extends Command {

  /**
   * Selects which motion-compensation algorithm is used when {@link ShootMode#SCORE} is active.
   *
   * <ul>
   *   <li>{@code DIRECT} — Method A: looks up RPS/hood from the radial-velocity 2D map at the
   *       <em>current</em> distance, then applies a simple trigonometric yaw lead. Fast and
   *       predictable, but does not account for where the robot will be at note landing.
   *   <li>{@code LOOKAHEAD} — Method B: iterative solver that finds the "virtual target" the turret
   *       must aim at so the note reaches the real target after the robot has moved. RPS and hood
   *       are read from the static-shot map at the converged lookahead distance.
   * </ul>
   *
   * Hot-switching between modes is <strong>not</strong> supported; the mode is fixed at command
   * construction time.
   */
  public enum MotionShotMode {
    DIRECT,
    LOOKAHEAD
  }

  private final Drive drive = Drive.getInstance();
  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final TurretSubsystem turret = TurretSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final FeederSubsystem feeder = FeederSubsystem.getInstance();
  private final ImprovedCommandXboxController operatorController =
      RobotContainer.operatorController;
  private final Debouncer shooterDebouncer = new Debouncer(0.12, DebounceType.kFalling);
  private final Button shootButton;
  private final ShootMode shootMode;
  private final MotionShotMode motionShotMode;
  private final boolean autoTriggerEnabled;
  private final TrajectorySimulator trajectorySimulator = new TrajectorySimulator();
  private final SwerveDriveSimulation driveSimulation = RobotContainer.driveSimulation;

  /** Persistent RPS offset applied on top of the solver result. Adjusted via ABXY. */
  private double rpsOffset = 0.0;

  /**
   * Target angular velocity feedforward for the turret (deg/s). Computed each execute() cycle as:
   * tangentialVelocityToVirtualTarget / lookaheadDistance. Passed into the turret subsystem via a
   * DoubleSupplier so the velocity loop always reads the latest value.
   */
  private double targetVelFFDegsPerSec = 0.0;

  // private static final double LEAD_YAW_COMPENSATION_INDEX = 6.;

  /**
   * Creates a HybridShootCommand using the default motion-shot algorithm ({@link
   * MotionShotMode#LOOKAHEAD}).
   */
  public HybridShootCommand(Button shootButton, ShootMode shootMode) {
    this(shootButton, shootMode, MotionShotMode.LOOKAHEAD, false);
  }

  /**
   * Creates a HybridShootCommand using the default motion-shot algorithm ({@link
   * MotionShotMode#LOOKAHEAD}) with optional automatic trigger behavior.
   */
  public HybridShootCommand(Button shootButton, ShootMode shootMode, boolean autoTriggerEnabled) {
    this(shootButton, shootMode, MotionShotMode.LOOKAHEAD, autoTriggerEnabled);
  }

  /**
   * Creates a HybridShootCommand with an explicit motion-shot algorithm selection.
   *
   * @param shootButton operator button that must be held to actually fire
   * @param shootMode {@link ShootMode#SCORE} to aim at the hub, {@link ShootMode#PASS} for tower
   * @param motionShotMode which motion-compensation algorithm to apply in SCORE mode
   */
  public HybridShootCommand(
      Button shootButton, ShootMode shootMode, MotionShotMode motionShotMode) {
    this(shootButton, shootMode, motionShotMode, false);
  }

  /**
   * Creates a HybridShootCommand with explicit motion-shot algorithm selection and optional
   * automatic trigger behavior.
   *
   * @param shootButton operator button used for manual spin-up
   * @param shootMode {@link ShootMode#SCORE} to aim at the hub, {@link ShootMode#PASS} for tower
   * @param motionShotMode which motion-compensation algorithm to apply in SCORE mode
   * @param autoTriggerEnabled when true, spins up immediately and auto-feeds once all shot targets
   *     are within tolerance
   */
  public HybridShootCommand(
      Button shootButton,
      ShootMode shootMode,
      MotionShotMode motionShotMode,
      boolean autoTriggerEnabled) {
    addRequirements(hood, turret, shooter, feeder);
    this.shootButton = shootButton;
    this.shootMode = shootMode;
    this.motionShotMode = motionShotMode;
    this.autoTriggerEnabled = autoTriggerEnabled;
  }

  @Override
  public void initialize() {
    hood.setModeHybrid();
    turret.setModeVelocity(drive::getRobotOmegaRadPerSec, () -> targetVelFFDegsPerSec);
    hood.setOperatorInputScalar(0.0);
    turret.setOperatorInputScalar(0.0);
    rpsOffset = 0.0;
  }

  @Override
  public void execute() {
    double distanceMeters = 0.0;
    double radialVelocity = 0.0;
    double tangentialVelocity = 0.0;
    Rotation2d fieldTargetAngle;
    Rotation2d straightToTarget; // uncompensated angle directly at the real target
    double targetRps;
    double hoodDegs;

    if (shootMode == ShootMode.SCORE) {
      Translation2d turretPos = drive.getTurretWorldPosition();
      Translation2d hubCenter = Drive.getAllianceHubCenter();
      Translation2d turretVel = drive.getTurretFieldVelocity();
      straightToTarget = drive.getRotationToAllianceHub();

      // Decompose field-frame velocity into hub-relative (radial, tangential) for logging
      // and for Method A's 2D map lookup.
      Translation2d turretToHub = hubCenter.minus(turretPos);
      double dist = turretToHub.getNorm();
      if (dist > 1e-6) {
        Translation2d radialUnit = turretToHub.div(dist);
        Translation2d tangentialUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX());
        radialVelocity =
            turretVel.getX() * radialUnit.getX() + turretVel.getY() * radialUnit.getY();
        tangentialVelocity =
            turretVel.getX() * tangentialUnit.getX() + turretVel.getY() * tangentialUnit.getY();
      }

      if (motionShotMode == MotionShotMode.LOOKAHEAD) {
        // ── Method B: iterative lookahead solver ────────────────────────────
        ShotSolution sol = ProjectileCalculator.solve(turretPos, hubCenter, turretVel);
        distanceMeters = sol.lookaheadDistance();
        targetRps = sol.shooterRps();
        hoodDegs = sol.hoodAngleDeg();
        fieldTargetAngle = sol.turretAimAngle();

        // Target angular velocity FF: how fast the turret must rotate to keep tracking the
        // virtual target as the robot moves.
        //   ω (rad/s) = tangentialVelocity / distance  →  deg/s = toDegrees(ω)
        double tangentialToVirtual = drive.getTurretTangentialVelocityToTarget(sol.virtualTarget());
        targetVelFFDegsPerSec =
            distanceMeters > 1e-6
                ? Math.toDegrees(Math.atan(tangentialToVirtual / distanceMeters))
                : 0.0;

        Logger.recordOutput(
            "Cmds/HybridShoot/VirtualTarget", new Pose2d(sol.virtualTarget(), new Rotation2d()));
        Logger.recordOutput("Cmds/HybridShoot/FlightTimeSecs", sol.flightTimeSecs());
        Logger.recordOutput("Cmds/HybridShoot/TangentialToVirtualMPS", tangentialToVirtual);
        Logger.recordOutput("Cmds/HybridShoot/TargetVelFFDegsPerSec", targetVelFFDegsPerSec);
      } else {
        // ── Method A: direct 2D map lookup ──────────────────────────────────
        distanceMeters = dist;
        targetRps =
            ProjectileCalculator.estimateMotionShotRps_Direct(distanceMeters, radialVelocity);
        hoodDegs =
            ProjectileCalculator.estimateMotionShotHoodAngle_Direct(distanceMeters, radialVelocity);
        double leadYawDegs =
            ProjectileCalculator.estimateLeadYawDegrees_Direct(distanceMeters, tangentialVelocity);
        fieldTargetAngle = straightToTarget.plus(Rotation2d.fromDegrees(leadYawDegs));
        // Method A does not produce a virtual target, so no angular FF is available.
        targetVelFFDegsPerSec = 0.0;
      }
    } else {
      // ── Pass mode: shoot to the nearest bump ──────────────────────────────
      //
      // 1. Auto-select bump target based on robot Y vs field center.
      //    Robot Y > fieldCenter → left side of field → aim at left bump.
      //    Robot Y ≤ fieldCenter → right side of field → aim at right bump.
      double fieldCenterY = FieldConstants.fieldWidth / 2.0;

      // Compute bump center Translation2d from LinesHorizontal bounds.
      // Red alliance uses the opposing hub center X.
      boolean isBlue = DriverStation.getAlliance().get() == Alliance.Blue;
      double bumpX =
          isBlue
              ? FieldConstants.LinesVertical.hubCenter
              : FieldConstants.LinesVertical.oppHubCenter;

      Translation2d bumpTarget;
      if (drive.getTurretWorldPosition().getY() > fieldCenterY) {
        // Left side — aim at left bump center
        double leftBumpCenterY =
            (FieldConstants.LinesHorizontal.leftBumpStart
                    + FieldConstants.LinesHorizontal.leftBumpEnd)
                / 2.0;
        bumpTarget = new Translation2d(bumpX, leftBumpCenterY);
      } else {
        // Right side — aim at right bump center
        double rightBumpCenterY =
            (FieldConstants.LinesHorizontal.rightBumpStart
                    + FieldConstants.LinesHorizontal.rightBumpEnd)
                / 2.0;
        bumpTarget = new Translation2d(bumpX, rightBumpCenterY);
      }

      // 2. Compute actual distance to the chosen bump for RPS scaling.
      distanceMeters = drive.getDistanceToTarget(bumpTarget);
      straightToTarget = bumpTarget.minus(drive.getTurretWorldPosition()).getAngle();

      // Linear RPS extrapolation:  adjustedRps = PassRps + slope * (distance - neutralDistance)
      targetRps =
          ShooterConstants.PassRps
              + ShooterConstants.PassRpsPerMeter
                  * (distanceMeters - ShooterConstants.PassRpsNeutralDistanceMeters);
      hoodDegs = HoodConstants.PassHoodDegs;

      // 3. Simple yaw lead to compensate for chassis lateral motion.
      //    leadDeg = tangentialVelocity * PassLeadIndex / distanceMeters (rad) → degrees
      double tangentialToBump = drive.getTurretTangentialVelocityToTarget(bumpTarget);
      double leadRad =
          distanceMeters > 1e-6
              ? Math.atan(tangentialToBump * ShooterConstants.PassLeadIndex / distanceMeters)
              : 0.0;
      fieldTargetAngle = straightToTarget.plus(Rotation2d.fromRadians(leadRad));

      tangentialVelocity = tangentialToBump;
      radialVelocity = drive.getTargetRelativeChassisSpeeds(bumpTarget).getX();

      // Pass mode does not use the turret angular velocity FF.
      targetVelFFDegsPerSec = Math.toDegrees(Math.atan(tangentialToBump / distanceMeters));

      Logger.recordOutput(
          "Cmds/HybridShoot/PassBumpTarget", new Pose2d(bumpTarget, new Rotation2d()));
      Logger.recordOutput("Cmds/HybridShoot/PassLeadDegs", Math.toDegrees(leadRad));
    }

    hood.setAutoSetpoint(hoodDegs);
    turret.setAutoSetpointFieldRelativeRotation2d(fieldTargetAngle, drive.getPose());

    hood.setOperatorInputScalar(
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getLeftY()));
    turret.setOperatorInputScalar(
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getRightX()));

    // ABXY adjust the persistent RPS offset: B=-1, A=-0.5, X=+0.5, Y=+1
    if (operatorController.getButtonPressed(Button.kB)) rpsOffset = -0.5;
    if (operatorController.getButtonPressed(Button.kA)) rpsOffset = -1.0;
    if (operatorController.getButtonPressed(Button.kX)) rpsOffset = 0.5;
    if (operatorController.getButtonPressed(Button.kY)) rpsOffset = 1.0;
    double adjustedTargetRps = targetRps + rpsOffset;

    boolean shootingEnabled = autoTriggerEnabled || operatorController.getButton(shootButton);
    if (shootingEnabled) {
      shooter.setRPS(adjustedTargetRps);
    } else {
      shooter.stop();
    }

    boolean hoodAtTarget = hood.isAtTargetPosition();
    boolean turretAtTarget = turret.isAtTargetPosition(distanceMeters);
    boolean shooterAtTarget = shooterDebouncer.calculate(shooter.isAtTargetRps());
    boolean distanceInScope = distanceMeters <= 5.3 && distanceMeters >= 1;
    boolean readyToAutoFeed = shooterAtTarget && turretAtTarget && hoodAtTarget && distanceInScope;
    boolean feedingEnabled = readyToAutoFeed || operatorController.getButton(Button.kRightBumper);

    // Sim-only: emit trajectory every frame using calculated aim parameters.
    // This shows where the shooter is aiming based on current distance/velocity calculations.
    if (Constants.currentMode == Constants.Mode.SIM) {
    // Log the simulated pose we will send to the trajectory viz, then emit trajectory.
    Logger.recordOutput(
      "Cmds/HybridShoot/SimPose", driveSimulation.getSimulatedDriveTrainPose());
    trajectorySimulator.setTrajectory(
      driveSimulation.getSimulatedDriveTrainPose(), hoodDegs, adjustedTargetRps, fieldTargetAngle);
    }

    Logger.recordOutput("Cmds/HybridShoot/ShootMode", shootMode.toString());
    Logger.recordOutput("Cmds/HybridShoot/DistanceMeters", distanceMeters);
    Logger.recordOutput("Cmds/HybridShoot/RadialVelocityMPS", radialVelocity);
    Logger.recordOutput("Cmds/HybridShoot/TangentialVelocityMPS", tangentialVelocity);
    Logger.recordOutput("Cmds/HybridShoot/TargetRPS", targetRps);
    Logger.recordOutput("Cmds/HybridShoot/RpsOffset", rpsOffset);
    Logger.recordOutput("Cmds/HybridShoot/AdjustedTargetRPS", adjustedTargetRps);
    Logger.recordOutput("Cmds/HybridShoot/ActualRPS", shooter.getShooterRPS());
    Logger.recordOutput("Cmds/HybridShoot/DebouncedShooterAtTarget", shooterAtTarget);
    Logger.recordOutput("Cmds/HybridShoot/HoodDegs", hoodDegs);
    Logger.recordOutput("Cmds/HybridShoot/HoodAtTarget", hoodAtTarget);
    Logger.recordOutput(
        "Cmds/HybridShoot/LeadYawDegs", fieldTargetAngle.minus(straightToTarget).getDegrees());
    Logger.recordOutput("Cmds/HybridShoot/FieldTargetDegs", fieldTargetAngle.getDegrees());
    Logger.recordOutput("Cmds/HybridShoot/TurretRealPositionDegs", turret.getCurrentPositionDegs());
    Logger.recordOutput("Cmds/HybridShoot/TurretAtTarget", turretAtTarget);
    Logger.recordOutput("Cmds/HybridShoot/AutoTriggerEnabled", autoTriggerEnabled);
    Logger.recordOutput("Cmds/HybridShoot/ReadyToAutoFeed", readyToAutoFeed);
    Logger.recordOutput("Cmds/HybridShoot/ShootingEnabled", shootingEnabled);

    boolean reverseFeeder = operatorController.getHID().getPOV() == 90;

    if (reverseFeeder) {
      feeder.setTurntableRPS(FeederConstants.ReverseTurntableRPS);
      feeder.setFeedRPS(FeederConstants.ReverseFeederRPS);
    } else if (feedingEnabled) {
      feeder.setTurntableRPS(FeederConstants.DefaultTurntableRPS);
      feeder.setFeedRPS(FeederConstants.DefaultFeedRPS);
    } else {
      feeder.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    hood.setAutoSetpoint(HoodConstants.IdlePosition);
    hood.setOperatorInputScalar(0.0);
    turret.setOperatorInputScalar(0.0);
    shooter.stop();
    feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
