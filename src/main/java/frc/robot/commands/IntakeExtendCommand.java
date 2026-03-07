package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Stretcher.StretcherSubsystem;

/**
 * Instant command to extend intake and start spinning.
 * Sets the target position and RPS, then immediately finishes.
 * Designed for manual control (e.g., button triggers) or use in sequences.
 * Default command will maintain the extended state.
 */
public class IntakeExtendCommand extends InstantCommand {
  public IntakeExtendCommand() {
    super(
        () -> {
          IntakeSubsystem intake = IntakeSubsystem.getInstance();
          StretcherSubsystem stretcher = StretcherSubsystem.getInstance();
          stretcher.extend();
          intake.setRPS(Constants.IntakeConstants.IntakingRPS);
        },
        IntakeSubsystem.getInstance(),
        StretcherSubsystem.getInstance());
  }
}
