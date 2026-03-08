package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Stretcher.StretcherSubsystem;
import frc.robot.subsystems.SuperStructure;

public class IntakeDefaultCommand extends Command {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final StretcherSubsystem stretcher = StretcherSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();
  private double targetPosition = Constants.StretcherConstants.ExtendedPosition;
  private final Timer timer = new Timer();
  private double lastToggleTime = 0.0;

  public IntakeDefaultCommand() {
    addRequirements(intake, stretcher);
  }

  @Override
  public void initialize() {
    // start/reset timer used to toggle target position every 0.4s
    timer.reset();
    timer.start();
    lastToggleTime = timer.get();
    targetPosition = Constants.StretcherConstants.ExtendedPosition;
  }

  @Override
  public void execute() {
    SuperStructure.IntakeMode intakeMode = superStructure.getIntakeMode();
    
    switch (intakeMode) {
      case INTAKE:
        // Toggle between Extended and Mid every 0.7 seconds
        if (targetPosition == Constants.StretcherConstants.ExtendedPosition && timer.get() - lastToggleTime >= 1.) {
          lastToggleTime = timer.get();
          targetPosition = Constants.StretcherConstants.MidPosition;
        } else if (targetPosition == Constants.StretcherConstants.MidPosition && timer.get() - lastToggleTime >= 0.5) {
          lastToggleTime = timer.get();
          targetPosition = Constants.StretcherConstants.ExtendedPosition;
        }
        stretcher.setPosition(targetPosition);
        // keep running intake while intaking
        intake.setRPS(Constants.IntakeConstants.IntakingRPS);
        break;
        
      case OFF:
        // Intake retract logic: command retract and only stop intake after
        // the stretcher has reached the retracted position.
        stretcher.setPosition(Constants.StretcherConstants.RetractedPosition);
        // Keep intake running while retracting so cargo is pulled in, then stop
        // once the stretcher reports being at target.
        if (!stretcher.isAtTargetPosition()) {
          intake.setRPS(Constants.IntakeConstants.IntakingRPS);
        } else {
          intake.stop();
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // ensure timer stopped and intake not left running
    timer.stop();
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

