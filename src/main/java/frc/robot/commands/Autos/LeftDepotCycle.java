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

public class LeftDepotCycle extends SequentialCommandGroup {
  Drive drive = Drive.getInstance();
  SuperStructure superStructure = SuperStructure.getInstance();

  public LeftDepotCycle() {
    // if (DriverStation.getAlliance().isPresent()
    //     && DriverStation.getAlliance().get() == Alliance.Blue) {
    //   addCommands(
    //       new InstantCommand(
    //           () ->
    //               drive.setPose(
    //                   drive.generatePPPath("Left-LeftNA").getStartingHolonomicPose().get())));
    // } else {
    //   addCommands(
    //       new InstantCommand(
    //           () ->
    //               drive.setPose(
    //                   drive
    //                       .generatePPPath("Left-LeftNA")
    //                       .flipPath()
    //                       .getStartingHolonomicPose()
    //                       .get())));
    // }
    addCommands(
        new InstantCommand(
            () -> {
              if (DriverStation.getAlliance().get() == Alliance.Blue) {
                drive.setPose(drive.generatePPPath("Left-LeftNA").getStartingHolonomicPose().get());
              } else {
                drive.setPose(
                    drive
                        .generatePPPath("Left-LeftNA")
                        .flipPath()
                        .getStartingHolonomicPose()
                        .get());
              }
            }));
    addCommands(
        drive
            .followPPPath("Left-LeftNA")
            .alongWith(
                superStructure.runOnce(
                    () -> superStructure.setIntakeMode(SuperStructure.IntakeMode.INTAKE))));
    addCommands(drive.followPPPath("LeftNA-Left"));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE).withTimeout(4.));
    addCommands(drive.followPPPath("Left-Depot"));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE));
  }
}
