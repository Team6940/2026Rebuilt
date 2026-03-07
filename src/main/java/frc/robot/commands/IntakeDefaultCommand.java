package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Stretcher.StretcherSubsystem;
import frc.robot.subsystems.SuperStructure;

public class IntakeDefaultCommand extends Command {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final StretcherSubsystem stretcher = StretcherSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();
  private double targetPosition = Constants.StretcherConstants.ExtendedPosition;

  public IntakeDefaultCommand() {
    addRequirements(intake, stretcher);
  }

  @Override
  public void execute() {
    SuperStructure.IntakeMode intakeMode = superStructure.getIntakeMode();
    
    switch (intakeMode) {
      case INTAKE:
        // Intake extend logic with position switching
        if (stretcher.getPosition() > Constants.StretcherConstants.ExtendedPosition
            - Constants.StretcherConstants.StretcherPositionToleranceRotations) {
          targetPosition = Constants.StretcherConstants.MidPosition;
        } else if (stretcher.getPosition() < Constants.StretcherConstants.MidPosition
            + Constants.StretcherConstants.StretcherPositionToleranceRotations) {
          targetPosition = Constants.StretcherConstants.ExtendedPosition;
        }
        stretcher.setPosition(targetPosition);
        intake.setRPS(Constants.IntakeConstants.IntakingRPS);
        break;
        
      case OFF:
        // Intake retract logic
        stretcher.setPosition(Constants.StretcherConstants.RetractedPosition);
        intake.stop();
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

