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
  private double lastTurretSetpointDegs = TurretConstants.IdlePosition;
  private static final double JOYSTICK_DEADZONE = 0.1;

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
    turret.setOperatorInputScalar(0.0);
    lastTurretSetpointDegs = turret.getCurrentPositionDegs();
    turret.setManualSetpoint(lastTurretSetpointDegs);
  }

  @Override
  public void execute() {
    hood.setOperatorInputScalar(-operatorController.getRightY());

    double leftX = operatorController.getLeftX();
    double leftY = operatorController.getLeftY();
    if (Math.hypot(leftX, leftY) >= JOYSTICK_DEADZONE) {
      // Joystick angle is field-relative (0 deg is field "up", clockwise positive).
      double desiredFieldAngle =
          operatorController.getJoystickAngleDeg180(leftX, leftY, JOYSTICK_DEADZONE);
      // Convert to turret-relative angle by subtracting robot heading.
      double turretSetpoint =
          turretSubsystem.findNearestEquivalentAngle(
              desiredFieldAngle - drive.getPose().getRotation().getDegrees(),
              turretSubsystem.getCurrentPositionDegs(),
              TurretConstants.MinDegs,
              TurretConstants.MaxDegs);
      lastTurretSetpointDegs = turretSetpoint;
    }
    turret.setManualSetpoint(lastTurretSetpointDegs);
    if (operatorController.getButtonPressed(Button.kA)) {
      targetRps = ShooterConstants.ManualRpsA;
    } else if (operatorController.getButtonPressed(Button.kB)) {
      targetRps = ShooterConstants.ManualRpsB;
    } else if (operatorController.getButtonPressed(Button.kX)) {
      targetRps = ShooterConstants.ManualRpsX;
    } else if (operatorController.getButtonPressed(Button.kY)) {
      targetRps = ShooterConstants.ManualRpsY;
    }

    if (operatorController.getButton(Button.kLeftBumper)) {
      targetRps += 5.0;
    } else if (operatorController.getButton(Button.kLeftTrigger)) {
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
      hood.setManualSetpoint(HoodConstants.IdlePosition); //0.
      turret.setManualSetpoint(TurretConstants.IdlePosition); //0.
      hood.setOperatorInputScalar(0.0);
      turret.setOperatorInputScalar(0.0);
      lastTurretSetpointDegs = TurretConstants.IdlePosition;
    }
  }

  @Override
  public void end(boolean interrupted) {
    hood.setOperatorInputScalar(0.0);
    turret.setOperatorInputScalar(0.0);
    turret.setManualSetpoint(TurretConstants.IdlePosition); //0.
    shooter.stop();
    feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
