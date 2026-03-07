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

public class RightNA extends SequentialCommandGroup {
  Drive drive = Drive.getInstance();

  public RightNA() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive.generatePPPath("Right-RightNA").getStartingHolonomicPose().get())));
    } else {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive
                          .generatePPPath("Right-RightNA")
                          .flipPath()
                          .getStartingHolonomicPose()
                          .get())));
    }

    addCommands(
        drive
            .followPPPath("Right-RightNA")
            .deadlineFor(new WaitCommand(1.).andThen(new IntakeCommand())));
    addCommands(drive.followPPPath("RightNA-Right"));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
  }
}
