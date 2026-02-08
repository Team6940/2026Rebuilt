package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.HybridShootCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure.ShootMode;

public class Mid extends SequentialCommandGroup {
  Drive drive = Drive.getInstance();

  public Mid() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive.generatePPPath("Mid-LeftC").getStartingHolonomicPose().get())));
    } else {
      addCommands(
          new InstantCommand(
              () ->
                  drive.setPose(
                      drive
                          .generatePPPath("Mid-LeftC")
                          .flipPath()
                          .getStartingHolonomicPose()
                          .get())));
    }

    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
    addCommands(drive.followPPPath("Mid-LeftC"));
  }
}
