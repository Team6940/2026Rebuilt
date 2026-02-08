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

public class Right_2Cycles extends SequentialCommandGroup {
  Drive drive = Drive.getInstance();

  public Right_2Cycles() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive.generatePPPath("Right-RightN1").getStartingHolonomicPose().get())));
    } else {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive
                          .generatePPPath("Right-RightN1")
                          .flipPath()
                          .getStartingHolonomicPose()
                          .get())));
    }

    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
    addCommands(
        drive
            .followPPPath("Right-RightN1")
            .alongWith(
                new WaitCommand(1.)
                    .andThen(new IntakeCommand().withDeadline(drive.followPPPath("Right-RightN1")))));
    addCommands(drive.followPPPath("RightN1-RightS"));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
    addCommands(
        drive
            .followPPPath("RightS-RightN2")
            .alongWith(
                new WaitCommand(1.)
                    .andThen(
                        new IntakeCommand().withDeadline(drive.followPPPath("RightS-RightN2")))));
    addCommands(drive.followPPPath("RightN2-RightS"));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
    addCommands(drive.followPPPath("RightS-RightC"));
  }
}
