package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Stretcher.StretcherSubsystem;
import java.util.Set;
import frc.robot.commands.HybridShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeExtendCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.ManualShootFieldRelativeCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.ClimbExtendCommand;
import frc.robot.commands.ClimbRetractCommand;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private static SuperStructure instance;

  public static SuperStructure getInstance() {
    return instance == null ? (instance = new SuperStructure()) : instance;
  }

  public enum ControlMode {
    HYBRID,
    MANUAL
  }

  public enum ShootMode {
    SCORE,
    PASS
  }

  public enum IntakeMode {
    INTAKE,
    OFF
  }

  private ControlMode controlMode = ControlMode.HYBRID;
  private ShootMode shootMode = ShootMode.SCORE;
  private IntakeMode intakeMode = IntakeMode.OFF;
  
  // currently scheduled intake-related command (if any)
  private Command currentIntakeCommand = null;
  public void setControlMode(ControlMode mode) {
    controlMode = mode;
  }

  public void setShootMode(ShootMode mode) {
    shootMode = mode;
  }

  public void setIntakeMode(IntakeMode mode) {
    intakeMode = mode;
  }

  public void toggleControlMode() {
    controlMode = controlMode == ControlMode.HYBRID ? ControlMode.MANUAL : ControlMode.HYBRID;
  }

  public void toggleShootMode() {
    shootMode = shootMode == ShootMode.SCORE ? ShootMode.PASS : ShootMode.SCORE;
  }

  public void toggleIntakeMode() {
    // flip state
    intakeMode = intakeMode == IntakeMode.INTAKE ? IntakeMode.OFF : IntakeMode.INTAKE;

    // cancel previously scheduled intake command (if any)
    if (currentIntakeCommand != null) {
      CommandScheduler.getInstance().cancel(currentIntakeCommand);
      currentIntakeCommand = null;
    }

    // create deferred command for the new mode and schedule it
    Command deferred =
        Commands.defer(() -> getIntakeCommand(), Set.of(IntakeSubsystem.getInstance(), StretcherSubsystem.getInstance()));
    currentIntakeCommand = deferred;
    CommandScheduler.getInstance().schedule(deferred);
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public ShootMode getShootMode() {
    return shootMode;
  }

  public IntakeMode getIntakeMode() {
    return intakeMode;
  }

  public Command getManualShootCommand(Button shootButton, Button resetButton) {
    //return new ManualShootCommand(shootButton, resetButton);
    return new ManualShootFieldRelativeCommand(shootButton, resetButton);
  }

  public Command getHybridShootCommand(Button shootButton) {
    return new HybridShootCommand(shootButton, shootMode);
  }

  public Command getShootCommand(Button shootButton, Button resetButton) {
    return switch (controlMode) {
      case HYBRID -> getHybridShootCommand(shootButton);
      case MANUAL -> getManualShootCommand(shootButton, resetButton);
    };
  }

  public Command getIntakeCommand() {
    return switch (intakeMode) {
      case INTAKE -> new IntakeExtendCommand();
      case OFF -> new IntakeRetractCommand();
    };
  }

  public Command getClimbExtendCommand() {
    return new ClimbExtendCommand();
  }
  public Command getClimbRetractCommand() {
    return new ClimbRetractCommand();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SuperStructure/ControlMode", controlMode);
    Logger.recordOutput("SuperStructure/ShootMode", shootMode);
    Logger.recordOutput("SuperStructure/IntakeMode", intakeMode);
    // Ensure the currentIntakeCommand reference matches scheduler state and
    // restore a command for the current intakeMode if none is scheduled.
    if (currentIntakeCommand != null) {
      if (!CommandScheduler.getInstance().isScheduled(currentIntakeCommand)) {
        // command finished or was cancelled; clear reference so we can restore
        currentIntakeCommand = null;
      }
    }

    if (currentIntakeCommand == null) {
      // no command currently tracked; schedule one for the current intake mode
      Command deferred =
          Commands.defer(
              () -> getIntakeCommand(),
              Set.of(IntakeSubsystem.getInstance(), StretcherSubsystem.getInstance()));
      currentIntakeCommand = deferred;
      CommandScheduler.getInstance().schedule(deferred);
    }
  }
}
