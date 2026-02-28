package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import org.littletonrobotics.junction.Logger;

public class ManualShootFieldRelativeCommand extends Command {
  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final TurretSubsystem turret = TurretSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final FeederSubsystem feeder = FeederSubsystem.getInstance();
  private final Drive drive = Drive.getInstance();

  private final ImprovedCommandXboxController operatorController =
      RobotContainer.operatorController;

  private double targetRps = ShooterConstants.ManualRpsY;
  private final Button shootButton;
  private final Button resetButton;
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
    turret.setOperatorInputScalar(
        0.); // This is for safety reasons. Generally we CANNOT directly control setpoints, instead,
    // setpoints should only be decided inside the subsystem itself by using set scalar.
    // In this situation, we manage setpoints directly so scalar should ALWAYS be 0.
    hood.setOperatorInputScalar(
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getLeftY()));

    double controllerX = -operatorController.getRightX();
    double controllerY = -operatorController.getRightY();
    if (Math.hypot(controllerX, controllerY) >= JOYSTICK_DEADZONE) {
      // Joystick angle is field-relative (0 deg is field "up", clockwise positive).
      double desiredFieldAngle =
          operatorController.getJoystickAngleDeg180(controllerX, controllerY, JOYSTICK_DEADZONE);
      // Flip 180° for Red alliance so joystick "up" still points away from our hub.
      boolean isRed =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
      if (isRed) {
        desiredFieldAngle += 180.0;
      }
      // Convert to turret-relative angle by subtracting robot heading.
      double turretSetpoint =
          turret.findNearestEquivalentAngle(
              desiredFieldAngle - drive.getPose().getRotation().getDegrees(),
              turret.getCurrentPositionDegs(),
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

    if (operatorController.getButton(shootButton)) {
      shooter.setRPS(targetRps);
      feeder.setRPS(FeederConstants.DefaultTurntableRPS, FeederConstants.DefaultFeedRPS);
    } else {
      shooter.stop();
      feeder.stop();
    }
    if (operatorController.getButton(resetButton)) {
      hood.setManualSetpoint(HoodConstants.IdlePosition); // 0.
      turret.setManualSetpoint(TurretConstants.IdlePosition); // 0.
      hood.setOperatorInputScalar(0.0);
      turret.setOperatorInputScalar(0.0);
      lastTurretSetpointDegs = TurretConstants.IdlePosition;
    }

    Logger.recordOutput("Cmds/ManualShootFieldRelative/TargetRPS", targetRps);
    Logger.recordOutput("Cmds/ManualShootFieldRelative/ActualRPS", shooter.getShooterRPS());
    Logger.recordOutput("Cmds/ManualShootFieldRelative/ShooterAtTarget", shooter.isAtTargetRps());
    Logger.recordOutput(
        "Cmds/ManualShootFieldRelative/TurretSetpointDegs(ChosenDegs)", lastTurretSetpointDegs);
    Logger.recordOutput(
        "Cmds/ManualShootFieldRelative/TurretPositionDegs", turret.getCurrentPositionDegs());
    Logger.recordOutput(
        "Cmds/ManualShootFieldRelative/TurretAtTarget", turret.isAtTargetPosition());
    Logger.recordOutput("Cmds/ManualShootFieldRelative/JoystickX", -operatorController.getRightX());
    Logger.recordOutput("Cmds/ManualShootFieldRelative/JoystickY", -operatorController.getRightY());
    Logger.recordOutput(
        "Cmds/ManualShootFieldRelative/ShootingEnabled", operatorController.getButton(shootButton));
    Logger.recordOutput(
        "Cmds/ManualShootFieldRelative/ResetPressed", operatorController.getButton(resetButton));
  }

  @Override
  public void end(boolean interrupted) {
    hood.setOperatorInputScalar(0.0);
    turret.setOperatorInputScalar(0.0);
    turret.setManualSetpoint(TurretConstants.IdlePosition);
    hood.setManualSetpoint(HoodConstants.IdlePosition);
    shooter.stop();
    feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
