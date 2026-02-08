package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HybridShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure.ShootMode;

public class Left_2Cycles extends SequentialCommandGroup {
  Drive drive = Drive.getInstance();

  public Left_2Cycles() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive.generatePPPath("Left-LeftN1").getStartingHolonomicPose().get())));
    } else {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive
                          .generatePPPath("Left-LeftN1")
                          .flipPath()
                          .getStartingHolonomicPose()
                          .get())));
    }

    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
    addCommands(
        drive
            .followPPPath("Left-LeftN1")
            .deadlineFor(new WaitCommand(1.).andThen(new IntakeCommand())));
    addCommands(drive.followPPPath("LeftN1-LeftS"));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
    addCommands(
        drive
            .followPPPath("LeftS-LeftN2")
            .deadlineFor(new WaitCommand(1.).andThen(new IntakeCommand())));
    addCommands(drive.followPPPath("LeftN2-LeftS"));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
    addCommands(drive.followPPPath("LeftS-LeftC"));
  }
}
