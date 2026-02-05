package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Stretcher.StretcherSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final StretcherSubsystem stretcher = StretcherSubsystem.getInstance();

  public IntakeCommand() {
    addRequirements(intake, stretcher);
  }

  @Override
  public void initialize() {
    intake();
  }

  @Override
  public void execute() {
    intake();
  }

  private void intake() {
    stretcher.setPosition(Constants.StretcherConstants.ExtendedPosition);
    intake.setRPS(Constants.IntakeConstants.IntakingRPS);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    stretcher.setPosition(Constants.StretcherConstants.RetractedPosition);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
