package frc.robot.commands;

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
import org.littletonrobotics.junction.Logger;

public class HybridShootCommand extends Command {
  private final Drive drive = Drive.getInstance();
  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final TurretSubsystem turret = TurretSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final FeederSubsystem feeder = FeederSubsystem.getInstance();
  private final ImprovedCommandXboxController operatorController =
      RobotContainer.operatorController;
  private final Button shootButton;
  private final ShootMode shootMode;

  private static final double LEAD_YAW_COMPENSATION_INDEX = 0.1;

  public HybridShootCommand(Button shootButton, ShootMode shootMode) {
    addRequirements(hood, turret, shooter, feeder);
    this.shootButton = shootButton;
    this.shootMode = shootMode;
  }

  @Override
  public void initialize() {
    hood.setModeHybrid();
    turret.setModeHybrid();
    hood.setOperatorInputScalar(0.0);
    turret.setOperatorInputScalar(0.0);
  }

  @Override
  public void execute() {
    double distanceMeters;
    Translation2d relativeSpeeds;
    double radialVelocity;
    double tangentialVelocity;
    Rotation2d fieldTargetAngle;
    double targetRps;
    double hoodDegs;
    double leadYawDegs;

    if (shootMode == ShootMode.SCORE) {
      // Score mode: shoot to hub with full motion compensation
      distanceMeters = drive.getDistanceToAllianceHub();
      relativeSpeeds = drive.getHubRelativeChassisSpeeds();
      radialVelocity = relativeSpeeds.getX();
      tangentialVelocity = relativeSpeeds.getY();

      targetRps = ProjectileCalculator.estimateMotionShotRps(distanceMeters, radialVelocity);
      hoodDegs = ProjectileCalculator.estimateMotionShotHoodAngle(distanceMeters, radialVelocity);
      leadYawDegs = ProjectileCalculator.estimateLeadYawDegrees(distanceMeters, tangentialVelocity);

      fieldTargetAngle = drive.getRotationToAllianceHub().plus(Rotation2d.fromDegrees(leadYawDegs));
    } else {
      // Pass mode: shoot to tower with static shooter settings
      distanceMeters = drive.getDistanceToAllianceTower();
      relativeSpeeds = drive.getTowerRelativeChassisSpeeds();
      radialVelocity = relativeSpeeds.getX();
      tangentialVelocity = relativeSpeeds.getY();

      targetRps = ShooterConstants.PassRps;
      hoodDegs = HoodConstants.PassHoodDegs;
      leadYawDegs =
          LEAD_YAW_COMPENSATION_INDEX * tangentialVelocity; // No lead compensation for passing

      fieldTargetAngle =
          drive.getRotationToAllianceTower().plus(Rotation2d.fromDegrees(leadYawDegs));
    }

    hood.setAutoSetpoint(hoodDegs);
    turret.setAutoSetpointFieldRelativeRotation2d(fieldTargetAngle, drive.getPose());

    hood.setOperatorInputScalar(
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getLeftY()));
    turret.setOperatorInputScalar(
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getRightX()));

    boolean shootingEnabled = operatorController.getButton(shootButton);
    Logger.recordOutput("Cmds/HybridShoot/ShootMode", shootMode.toString());
    Logger.recordOutput("Cmds/HybridShoot/DistanceMeters", distanceMeters);
    Logger.recordOutput("Cmds/HybridShoot/RadialVelocityMPS", radialVelocity);
    Logger.recordOutput("Cmds/HybridShoot/TangentialVelocityMPS", tangentialVelocity);
    Logger.recordOutput("Cmds/HybridShoot/TargetRPS", targetRps);
    Logger.recordOutput("Cmds/HybridShoot/ActualRPS", shooter.getShooterRPS());
    Logger.recordOutput("Cmds/HybridShoot/ShooterAtTarget", shooter.isAtTargetRps());
    Logger.recordOutput("Cmds/HybridShoot/HoodDegs", hoodDegs);
    Logger.recordOutput("Cmds/HybridShoot/LeadYawDegs", leadYawDegs);
    Logger.recordOutput("Cmds/HybridShoot/FieldTargetDegs", fieldTargetAngle.getDegrees());
    Logger.recordOutput("Cmds/HybridShoot/TurretRealPositionDegs", turret.getCurrentPositionDegs());
    Logger.recordOutput("Cmds/HybridShoot/TurretAtTarget", turret.isAtTargetPosition());
    Logger.recordOutput("Cmds/HybridShoot/ShootingEnabled", shootingEnabled);

    if (shootingEnabled) {
      shooter.setRPS(targetRps);
      feeder.setRPS(FeederConstants.DefaultTurntableRPS, FeederConstants.DefaultFeedRPS);
    } else {
      shooter.stop();
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
