package frc.robot.commands;

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
import frc.robot.subsystems.ImprovedCommandXboxController.Button;import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import org.littletonrobotics.junction.Logger;

public class ManualShootCommand extends Command {
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

  public ManualShootCommand(Button shootButton, Button resetButton) {
    addRequirements(hood, turret, shooter, feeder);
    this.shootButton = shootButton;
    this.resetButton = resetButton;
  }

  @Override
  public void initialize() {
    hood.setModeManual();
    turret.setModeManual();
    turret.setChassisOmegaSupplier(drive::getRobotOmegaRadPerSec);
    hood.setOperatorInputScalar(0.0);
    turret.setOperatorInputScalar(0.0);
  }

  @Override
  public void execute() {
    hood.setOperatorInputScalar(
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getLeftY()));
    turret.setOperatorInputScalar(
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getRightX()));
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
      hood.setManualSetpoint(HoodConstants.IdlePosition);
      turret.setManualSetpoint(TurretConstants.IdlePosition);
      hood.setOperatorInputScalar(0.0);
      turret.setOperatorInputScalar(0.0);
    }

    Logger.recordOutput("Cmds/ManualShoot/TargetRPS", targetRps);
    Logger.recordOutput("Cmds/ManualShoot/ActualRPS", shooter.getShooterRPS());
    Logger.recordOutput("Cmds/ManualShoot/ShooterAtTarget", shooter.isAtTargetRps());
    Logger.recordOutput("Cmds/ManualShoot/TurretPositionDegs", turret.getCurrentPositionDegs());
    Logger.recordOutput("Cmds/ManualShoot/TurretAtTarget", turret.isAtTargetPosition());
    Logger.recordOutput("Cmds/ManualShoot/HoodScalar", -operatorController.getLeftY());
    Logger.recordOutput("Cmds/ManualShoot/TurretScalar", -operatorController.getRightX());
    Logger.recordOutput(
        "Cmds/ManualShoot/ShootingEnabled", operatorController.getButton(shootButton));
    Logger.recordOutput("Cmds/ManualShoot/ResetPressed", operatorController.getButton(resetButton));
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
