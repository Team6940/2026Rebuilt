package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class ClimbExtendCommand extends Command {
  private final ClimberSubsystem climber = ClimberSubsystem.getInstance();
  private final TurretSubsystem turret = TurretSubsystem.getInstance();

  public ClimbExtendCommand() {
    addRequirements(climber, turret);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    turret.setManualSetpoint(90);
    climber.setExtended();
  }


  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
