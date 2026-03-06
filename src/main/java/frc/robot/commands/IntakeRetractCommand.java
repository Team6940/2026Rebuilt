package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Stretcher.StretcherSubsystem;

public class IntakeRetractCommand extends Command {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final StretcherSubsystem stretcher = StretcherSubsystem.getInstance();

  public IntakeRetractCommand() {
    addRequirements(intake, stretcher);
  }

  @Override
  public void execute() {
    stretcher.setPosition(Constants.StretcherConstants.RetractedPosition);
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
