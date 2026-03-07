package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Stretcher.StretcherSubsystem;
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
    if (instance == null) {
      instance = new SuperStructure();
      // Initialize default command for intake based on initial mode
      instance.updateIntakeDefaultCommand();
    }
    return instance;
  }
  
  private SuperStructure() {
    // Private constructor for singleton pattern
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
  
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  
  private void updateIntakeDefaultCommand() {
    // Get the new default command based on current intakeMode
    Command newDefaultCommand = getIntakeCommand();
    
    // Set the new default command on IntakeSubsystem
    // The framework will automatically cancel the old default command if it's running
    // This command will automatically run when no other command requires the intake subsystem
    intake.setDefaultCommand(newDefaultCommand);
  }
  
  public void setControlMode(ControlMode mode) {
    controlMode = mode;
  }

  public void setShootMode(ShootMode mode) {
    shootMode = mode;
  }

  public void setIntakeMode(IntakeMode mode) {
    if (intakeMode != mode) {
      intakeMode = mode;
      updateIntakeDefaultCommand();
    }
  }

  public void toggleControlMode() {
    controlMode = controlMode == ControlMode.HYBRID ? ControlMode.MANUAL : ControlMode.HYBRID;
  }

  public void toggleShootMode() {
    shootMode = shootMode == ShootMode.SCORE ? ShootMode.PASS : ShootMode.SCORE;
  }

  public void toggleIntakeMode() {
    // flip state
    IntakeMode newMode = intakeMode == IntakeMode.INTAKE ? IntakeMode.OFF : IntakeMode.INTAKE;
    setIntakeMode(newMode);
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
  }
}
