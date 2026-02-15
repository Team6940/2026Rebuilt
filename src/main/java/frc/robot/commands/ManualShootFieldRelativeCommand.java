package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class ManualShootFieldRelativeCommand extends Command {
  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final TurretSubsystem turret = TurretSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final FeederSubsystem feeder = FeederSubsystem.getInstance();
  private final Drive drive = Drive.getInstance();
  private final ImprovedCommandXboxController operatorController =
      RobotContainer.operatorController;
  private double targetRps = ShooterConstants.ManualRpsA;
  private final Button shootButton;
  private final Button resetButton;
  static TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();

  public ManualShootFieldRelativeCommand(Button shootButton, Button resetButton) {
    addRequirements(hood, turret, shooter, feeder);
    this.shootButton = shootButton;
    this.resetButton = resetButton;
  }

  @Override
  public void initialize() {
    hood.setModeManual();
    turret.setModeManual();
    hood.setOperatorInputScalar(0.0);
  }

  @Override
  public void execute() {
    hood.setOperatorInputScalar(-operatorController.getRightY());

    double desiredFieldAngle = operatorController.getJoystickAngleDeg180(operatorController.getLeftX(), operatorController.getLeftY(), 0.1);
    double turretSetpoint = turretSubsystem.findNearestEquivalentAngle(
      desiredFieldAngle - drive.getPose().getRotation().getDegrees(),
      // turret.getTurretFieldAngle(drive.getPose()).getDegrees(),
      turretSubsystem.getCurrentPositionDegs(),
      TurretConstants.MinDegs,
      TurretConstants.MaxDegs
);
    turret.setManualSetpoint(turretSetpoint);

    if (operatorController.getButtonPressed(Button.kA)) {
      targetRps = ShooterConstants.ManualRpsA;
    } else if (operatorController.getButtonPressed(Button.kB)) {
      targetRps = ShooterConstants.ManualRpsB;
    } else if (operatorController.getButtonPressed(Button.kX)) {
      targetRps = ShooterConstants.ManualRpsX;
    } else if (operatorController.getButtonPressed(Button.kY)) {
      targetRps = ShooterConstants.ManualRpsY;
    }

    if (operatorController.getButton(Button.kRightBumper)) {
      targetRps += 5.0;
    } else if (operatorController.getButton(Button.kRightTrigger)) {
      targetRps -= 5.0;
    }

    if (operatorController.getButton(shootButton)) {
      shooter.setRPS(targetRps);
      feeder.setRPS(FeederConstants.DefaultTurntableRPS, FeederConstants.DefaultFeedRPS);
    } else {
      shooter.stop();
      feeder.stop();
    }

    if (operatorController.getButton(resetButton)) {
      hood.setManualSetpoint(HoodConstants.IdlePosition);
      turret.setManualSetpoint(TurretConstants.IdlePosition);
      hood.setOperatorInputScalar(0.0);
      turret.setOperatorInputScalar(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    hood.setOperatorInputScalar(0.0);
    turret.setManualSetpoint(TurretConstants.IdlePosition);
    shooter.stop();
    feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}