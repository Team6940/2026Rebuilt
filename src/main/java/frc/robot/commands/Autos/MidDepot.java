package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HybridShootCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ShootMode;

public class MidDepot extends SequentialCommandGroup {
  Drive drive = Drive.getInstance();
  SuperStructure superStructure = SuperStructure.getInstance();

  public MidDepot() {
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
            .followPPPath("MidS-Depot")
            .alongWith(
                superStructure.runOnce(
                    () -> superStructure.setIntakeMode(SuperStructure.IntakeMode.INTAKE))));
    addCommands(
        new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE, true)
            .alongWith(
                new WaitCommand(1.5)
                    .andThen(
                        superStructure.runOnce(
                            () -> superStructure.setIntakeMode(SuperStructure.IntakeMode.SHAKE)))));
  }
}
