package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HybridShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualShootFieldRelativeCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.ClimbExtendCommand;
import frc.robot.commands.ClimbRetractCommand;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.SwerveDriveSendable;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.util.ManualShotRecommender;

import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private static SuperStructure instance;
  private Field2d field = new Field2d();
  private boolean swerveSendablePublished = false;

  public static SuperStructure getInstance() {
    return instance == null ? (instance = new SuperStructure()) : instance;
  }

  public enum ControlMode {
    HYBRID,
    MANUAL
  }

  public enum ShootMode {
    SCORE, //hub shot in hybrid & manual
    PASS, //tower shot in hybrid & manual
    FREE //not available in hybrid, any angle shot in manual
  }

  private ControlMode controlMode = ControlMode.HYBRID;
  private ShootMode shootMode = ShootMode.SCORE;

  public void setControlMode(ControlMode mode) {
    controlMode = mode;
    if (controlMode == ControlMode.HYBRID && shootMode == ShootMode.FREE) {
      shootMode = ShootMode.SCORE;
    }
  }

  public void setShootMode(ShootMode mode) {
    shootMode = mode;
  }

  public void toggleControlMode() {
    setControlMode(controlMode == ControlMode.HYBRID ? ControlMode.MANUAL : ControlMode.HYBRID);
  }

  /** the new shoot mode logic is such:
   * in HYBRID mode, the shoot button toggles between SCORE and PASS modes, and FREE mode is not accessible
   * in MANUAL mode, the shoot button toggles between SCORE, PASS, and FREE modes in a cycle 
   *   (SCORE -> PASS -> FREE -> SCORE, etc.)
   * toggling to HYBRID mode from MANUAL mode while in FREE shoot mode will
   * automatically switch to SCORE shoot mode since FREE mode is not applicable in HYBRID mode
   */
  public void toggleShootMode() {
    if (controlMode == ControlMode.HYBRID) {
      shootMode = shootMode == ShootMode.SCORE ? ShootMode.PASS : ShootMode.SCORE;
      return;
    }
    switch (shootMode) {
      case SCORE -> shootMode = ShootMode.PASS;
      case PASS -> shootMode = ShootMode.FREE;
      case FREE -> shootMode = ShootMode.SCORE;
    }
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public ShootMode getShootMode() {
    return shootMode;
  }

  public Command getManualShootCommand(Button shootButton, Button resetButton) {
    //return new ManualShootCommand(shootButton, resetButton);
    return new ManualShootFieldRelativeCommand(shootButton, resetButton);
  }

  public Command getHybridShootCommand(Button shootButton) {
    ShootMode effectiveMode = shootMode == ShootMode.FREE ? ShootMode.SCORE : shootMode;
    return new HybridShootCommand(shootButton, effectiveMode);
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

  public Command getClimbExtendCommand() {
    return new ClimbExtendCommand();
  }
  public Command getClimbRetractCommand() {
    return new ClimbRetractCommand();
  }

  @Override
  public void periodic() {
    Drive currentDrive = Drive.getInstance();
    if (currentDrive != null) {
      field.setRobotPose(currentDrive.getPose());
      if (!swerveSendablePublished) {
        SmartDashboard.putData("Swerve Drive", new SwerveDriveSendable(currentDrive));
        swerveSendablePublished = true;
      }
    } else {
      // Drive instance is no longer available; remove stale sendable and reset flag.
      if (swerveSendablePublished) {
        SmartDashboard.delete("Swerve Drive");
        swerveSendablePublished = false;
      }
    }
    SmartDashboard.putData("SuperStructure/Field", field);
    SmartDashboard.putString("SuperStructure/ShootMode", shootMode.toString());
    if (controlMode == ControlMode.MANUAL && currentDrive != null) {
      ManualShotRecommender.updateSmartDashboard(
          currentDrive, TurretSubsystem.getInstance(), shootMode);
    }
    // ManualShotRecommender.updateSmartDashboard(currentDrive, TurretSubsystem.getInstance(), shootMode);
    Logger.recordOutput("SuperStructure/ControlMode", controlMode);
    // Logger.recordOutput("SuperStructure/ShootMode", shootMode);
  }
}
