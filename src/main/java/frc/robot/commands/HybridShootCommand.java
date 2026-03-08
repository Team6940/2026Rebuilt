package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
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
  private final Button shootButton;
  private final ShootMode shootMode;
  private final MotionShotMode motionShotMode;

  private static final double LEAD_YAW_COMPENSATION_INDEX = 6.;

  /**
   * Creates a HybridShootCommand using the default motion-shot algorithm ({@link
   * MotionShotMode#LOOKAHEAD}).
   */
  public HybridShootCommand(Button shootButton, ShootMode shootMode) {
    this(shootButton, shootMode, MotionShotMode.LOOKAHEAD);
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
    addRequirements(hood, turret, shooter, feeder);
    this.shootButton = shootButton;
    this.shootMode = shootMode;
    this.motionShotMode = motionShotMode;
  }

  @Override
  public void initialize() {
    hood.setModeHybrid();
    turret.setModeVelocity(drive::getRobotOmegaRadPerSec);
    hood.setOperatorInputScalar(0.0);
    turret.setOperatorInputScalar(0.0);
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
        Logger.recordOutput(
            "Cmds/HybridShoot/VirtualTarget", new Pose2d(sol.virtualTarget(), new Rotation2d()));
        Logger.recordOutput("Cmds/HybridShoot/FlightTimeSecs", sol.flightTimeSecs());
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
      }
    } else {
      // Pass mode: shoot to tower with static shooter settings
      distanceMeters = drive.getDistanceToAllianceTower();
      straightToTarget = drive.getRotationToAllianceTower();
      Translation2d towerSpeeds = drive.getTowerRelativeChassisSpeeds();
      radialVelocity = towerSpeeds.getX();
      tangentialVelocity = towerSpeeds.getY();

      targetRps = ShooterConstants.PassRps;
      hoodDegs = HoodConstants.PassHoodDegs;
      double leadYawDegs = LEAD_YAW_COMPENSATION_INDEX * tangentialVelocity;
      fieldTargetAngle = straightToTarget.plus(Rotation2d.fromDegrees(leadYawDegs));
    }

    hood.setAutoSetpoint(hoodDegs);
    turret.setAutoSetpointFieldRelativeRotation2d(fieldTargetAngle, drive.getPose());

    hood.setOperatorInputScalar(
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getLeftY()));
    turret.setOperatorInputScalar(
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getRightX()));

    boolean shootingEnabled = operatorController.getButton(shootButton);
    boolean feedingEnabled = operatorController.getButton(Button.kRightBumper);
    Logger.recordOutput("Cmds/HybridShoot/ShootMode", shootMode.toString());
    Logger.recordOutput("Cmds/HybridShoot/DistanceMeters", distanceMeters);
    Logger.recordOutput("Cmds/HybridShoot/RadialVelocityMPS", radialVelocity);
    Logger.recordOutput("Cmds/HybridShoot/TangentialVelocityMPS", tangentialVelocity);
    Logger.recordOutput("Cmds/HybridShoot/TargetRPS", targetRps);
    Logger.recordOutput("Cmds/HybridShoot/ActualRPS", shooter.getShooterRPS());
    Logger.recordOutput("Cmds/HybridShoot/ShooterAtTarget", shooter.isAtTargetRps());
    Logger.recordOutput("Cmds/HybridShoot/HoodDegs", hoodDegs);
    Logger.recordOutput(
        "Cmds/HybridShoot/LeadYawDegs", fieldTargetAngle.minus(straightToTarget).getDegrees());
    Logger.recordOutput("Cmds/HybridShoot/FieldTargetDegs", fieldTargetAngle.getDegrees());
    Logger.recordOutput("Cmds/HybridShoot/TurretRealPositionDegs", turret.getCurrentPositionDegs());
    Logger.recordOutput("Cmds/HybridShoot/TurretAtTarget", turret.isAtTargetPosition());
    Logger.recordOutput("Cmds/HybridShoot/ShootingEnabled", shootingEnabled);

    if (shootingEnabled) {
      shooter.setRPS(targetRps);
    } else {
      shooter.stop();
    }

    if (feedingEnabled) {
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
