package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Stretcher.StretcherSubsystem;

public class IntakeExtendCommand extends Command {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final StretcherSubsystem stretcher = StretcherSubsystem.getInstance();
  private double targetPosition = Constants.StretcherConstants.ExtendedPosition;

  public IntakeExtendCommand() {
    addRequirements(intake, stretcher);
  }

  @Override
  public void execute() {
    if (stretcher.getPosition() > Constants.StretcherConstants.ExtendedPosition
        - Constants.StretcherConstants.StretcherPositionToleranceRotations) {
      targetPosition = Constants.StretcherConstants.MidPosition;
    } else if (stretcher.getPosition() < Constants.StretcherConstants.MidPosition
        + Constants.StretcherConstants.StretcherPositionToleranceRotations) {
      targetPosition = Constants.StretcherConstants.ExtendedPosition;
    }
    stretcher.setPosition(targetPosition);
    intake.setRPS(Constants.IntakeConstants.IntakingRPS);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
