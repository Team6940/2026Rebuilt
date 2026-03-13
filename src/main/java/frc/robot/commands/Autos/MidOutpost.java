package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.HybridShootCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ShootMode;

public class MidOutpost extends SequentialCommandGroup {
  Drive drive = Drive.getInstance();
  SuperStructure superStructure = SuperStructure.getInstance();

  public MidOutpost() {
    // if (DriverStation.getAlliance().isPresent()
    //     && DriverStation.getAlliance().get() == Alliance.Blue) {
    //   addCommands(
    //       new InstantCommand(
    //           () ->
    //               drive.setPose(
    //                   drive.generatePPPath("Mid-MidS").getStartingHolonomicPose().get())));
    // } else {
    //   addCommands(
    //       new InstantCommand(
    //           () ->
    //               drive.setPose(
    //                   drive
    //                       .generatePPPath("Mid-MidS")
    //                       .flipPath()
    //                       .getStartingHolonomicPose()
    //                       .get())));
    // }
    addCommands(
        new InstantCommand(
            () -> {
              if (DriverStation.getAlliance().get() == Alliance.Blue) {
                drive.setPose(drive.generatePPPath("Mid-MidS").getStartingHolonomicPose().get());
              } else {
                drive.setPose(
                    drive.generatePPPath("Mid-MidS").flipPath().getStartingHolonomicPose().get());
              }
            }));
    addCommands(drive.followPPPath("Mid-MidS"));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE, true).withTimeout(2.));
    addCommands(
        drive
            .followPPPath("MidS-Outpost")
            .alongWith(
                superStructure.runOnce(
                    () -> superStructure.setIntakeMode(SuperStructure.IntakeMode.SHAKE))));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE, true));
  }
}
