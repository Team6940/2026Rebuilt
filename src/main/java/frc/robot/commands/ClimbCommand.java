package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class ClimbCommand extends Command {
  private final ClimberSubsystem climber = ClimberSubsystem.getInstance();
  private final TurretSubsystem turret = TurretSubsystem.getInstance();

  public ClimbCommand() {
    addRequirements(climber, turret);
  }

  @Override
  public void initialize() {
    turret.setManualSetpoint(90);
    climber.setExtended();
  }

  @Override
  public void execute() {
    climb();
  }

  private void climb() {
    climber.setRetracted();
  }


  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
