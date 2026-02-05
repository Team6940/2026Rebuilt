package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
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

  public HybridShootCommand(Button shootButton) {
    addRequirements(hood, turret, shooter, feeder);
    this.shootButton = shootButton;
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
    double distanceMeters = drive.getDistanceToAllianceHub();
    Translation2d hubRelativeSpeeds = drive.getHubRelativeChassisSpeeds();
    double radialVelocity = hubRelativeSpeeds.getX();
    double tangentialVelocity = hubRelativeSpeeds.getY();

    double targetRps = ProjectileCalculator.estimateMotionShotRps(distanceMeters, radialVelocity);
    double hoodDegs =
        ProjectileCalculator.estimateMotionShotHoodAngle(distanceMeters, radialVelocity);
    double leadYawDegs =
        ProjectileCalculator.estimateLeadYawDegrees(distanceMeters, tangentialVelocity);

    Rotation2d fieldTargetAngle =
        drive.getRotationToAllianceHub().plus(Rotation2d.fromDegrees(leadYawDegs));

    hood.setAutoSetpoint(hoodDegs);
    turret.setAutoSetpointFieldRelativeRotation2d(fieldTargetAngle, drive.getPose());

    hood.setOperatorInputScalar(-operatorController.getLeftY());
    turret.setOperatorInputScalar(operatorController.getRightX());

    boolean shootingEnabled = operatorController.getButton(shootButton);
    Logger.recordOutput("HybridShoot/DistanceMeters", distanceMeters);
    Logger.recordOutput("HybridShoot/RadialVelocity", radialVelocity);
    Logger.recordOutput("HybridShoot/TangentialVelocity", tangentialVelocity);
    Logger.recordOutput("HybridShoot/TargetRps", targetRps);
    Logger.recordOutput("HybridShoot/HoodDegs", hoodDegs);
    Logger.recordOutput("HybridShoot/LeadYawDegs", leadYawDegs);
    Logger.recordOutput("HybridShoot/FieldTargetDegs", fieldTargetAngle.getDegrees());
    Logger.recordOutput("HybridShoot/ShootingEnabled", shootingEnabled);

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
