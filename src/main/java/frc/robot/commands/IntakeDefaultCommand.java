package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
  private final Timer timer = new Timer();
  private double lastToggleTime = 0.0;

  // Coast-hold state for INTAKE mode
  private boolean stretcherCoasting = false;
  private double outOfToleranceStartTime = -1.0;
  private SuperStructure.IntakeMode lastIntakeMode = null;

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
    stretcherCoasting = false;
    outOfToleranceStartTime = -1.0;
    lastIntakeMode = null;
  }

  @Override
  public void execute() {
    SuperStructure.IntakeMode intakeMode = superStructure.getIntakeMode();

    // Reset coast state whenever we freshly enter INTAKE mode from a different mode
    if (intakeMode == SuperStructure.IntakeMode.INTAKE
        && lastIntakeMode != SuperStructure.IntakeMode.INTAKE) {
      stretcherCoasting = false;
      outOfToleranceStartTime = -1.0;
    }
    lastIntakeMode = intakeMode;

    switch (intakeMode) {
      case INTAKE:
        intake.setRPS(Constants.IntakeConstants.IntakingRPS);
        if (!stretcherCoasting) {
          // Still driving to target - check if we've arrived
          stretcher.setPosition(Constants.StretcherConstants.ExtendedPosition);
          if (stretcher.isAtTargetPosition()) {
            stretcher.setCoast();
            stretcherCoasting = true;
            outOfToleranceStartTime = -1.0;
          }
        } else {
          // Coasting - watch for external disturbance pushing it out of tolerance
          if (!stretcher.isAtTargetPosition()) {
            if (outOfToleranceStartTime < 0.0) {
              outOfToleranceStartTime = timer.get();
            } else if (timer.get() - outOfToleranceStartTime > 1.0) {
              stretcherCoasting = false;
              outOfToleranceStartTime = -1.0;
            }
          } else {
            outOfToleranceStartTime = -1.0;
          }
        }
        break;

      case SHAKE:
        // Toggle between Extended and Mid every 0.7 seconds
        if (targetPosition == Constants.StretcherConstants.ExtendedPosition
            && timer.get() - lastToggleTime >= 0.25) {
          lastToggleTime = timer.get();
          targetPosition = Constants.StretcherConstants.MidPosition;
        } else if (targetPosition == Constants.StretcherConstants.MidPosition
            && timer.get() - lastToggleTime >= 0.25) {
          lastToggleTime = timer.get();
          targetPosition = Constants.StretcherConstants.ExtendedPosition;
        }
        stretcher.setPosition(targetPosition);
        // keep running intake while intaking
        intake.setRPS(Constants.IntakeConstants.IntakingRPS);
        break;

      case OFF:
        stretcher.setPosition(Constants.StretcherConstants.RetractedPosition);
        // if (stretcher.isAtTargetPosition()) {
        intake.stop();
        // }
        break;

      case STOPOUT:
        stretcher.setPosition(Constants.StretcherConstants.MidPosition);
        intake.setRPS(Constants.IntakeConstants.IntakingRPS);
        break;

      case REVERSE:
        stretcher.setPosition(Constants.StretcherConstants.ExtendedPosition);
        intake.setRPS(-Constants.IntakeConstants.IntakingRPS);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // ensure timer stopped and intake not left running
    timer.stop();
    intake.stop();
    stretcherCoasting = false;
    outOfToleranceStartTime = -1.0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
