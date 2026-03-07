package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class ClimbRetractCommand extends Command {
  private final ClimberSubsystem climber = ClimberSubsystem.getInstance();
  private final TurretSubsystem turret = TurretSubsystem.getInstance();
  private final ImprovedCommandXboxController driverController = RobotContainer.driverController;

  public ClimbRetractCommand() {
    addRequirements(climber, turret);
  }

  @Override
  public void initialize() {
    turret.setModeManual();
    turret.setManualSetpoint(90);
  }

  @Override
  public void execute() {
    climber.setRetracted();
    if (driverController.getButton(Button.kY)) {
      climber.setRotation(-1.2);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
