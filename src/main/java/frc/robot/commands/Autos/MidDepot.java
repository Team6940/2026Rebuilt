package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimbExtendCommand;
import frc.robot.commands.HybridShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure.ShootMode;

public class MidDepot extends SequentialCommandGroup {
  Drive drive = Drive.getInstance();

  public MidDepot() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive.generatePPPath("Mid-Depot").getStartingHolonomicPose().get())));
    } else {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive
                          .generatePPPath("Mid-Depot")
                          .flipPath()
                          .getStartingHolonomicPose()
                          .get())));
    }

    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
    addCommands(
        drive
            .followPPPath("Mid-Depot")
            .deadlineFor(new WaitCommand(1.).andThen(new IntakeCommand())));
    addCommands(drive.followPPPath("LeftDepot-LeftC"));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
    addCommands(new ClimbExtendCommand());
  }
}
