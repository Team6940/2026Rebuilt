package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimbExtendCommand;
import frc.robot.commands.ClimbRetractCommand;
import frc.robot.commands.HybridShootCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure.ShootMode;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class MidRC extends SequentialCommandGroup {
  Drive drive = Drive.getInstance();
  ClimberSubsystem climber = ClimberSubsystem.getInstance();
  TurretSubsystem turret = TurretSubsystem.getInstance();

  public MidRC() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive.generatePPPath("Mid-RightC").getStartingHolonomicPose().get())));
    } else {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive
                          .generatePPPath("Mid-RightC")
                          .flipPath()
                          .getStartingHolonomicPose()
                          .get())));
    }

    //addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE).withTimeout(3.));
    addCommands(drive.followPPPath("Mid-RightC").alongWith(new ClimbExtendCommand()));
    addCommands(new WaitCommand(1.));
    addCommands(new ClimbRetractCommand());
  }
}
