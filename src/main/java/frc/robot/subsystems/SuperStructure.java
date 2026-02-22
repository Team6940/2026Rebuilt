package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HybridShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualShootFieldRelativeCommand;
import frc.robot.commands.ManualShootCommand;
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

  private ControlMode controlMode = ControlMode.HYBRID;
  private ShootMode shootMode = ShootMode.SCORE;

  public void setControlMode(ControlMode mode) {
    controlMode = mode;
  }

  public void setShootMode(ShootMode mode) {
    shootMode = mode;
  }

  public void toggleControlMode() {
    controlMode = controlMode == ControlMode.HYBRID ? ControlMode.MANUAL : ControlMode.HYBRID;
  }

  public void toggleShootMode() {
    shootMode = shootMode == ShootMode.SCORE ? ShootMode.PASS : ShootMode.SCORE;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public ShootMode getShootMode() {
    return shootMode;
  }

  public Command getManualShootCommand(Button shootButton, Button resetButton) {
    return new ManualShootCommand(shootButton, resetButton);
    // return new ManualShootFieldRelativeCommand(shootButton, resetButton);
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
    return new IntakeCommand();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SuperStructure/ControlMode", controlMode);
    Logger.recordOutput("SuperStructure/ShootMode", shootMode);
  }
}
