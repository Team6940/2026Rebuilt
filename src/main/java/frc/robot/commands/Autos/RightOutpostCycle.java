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

public class RightOutpostCycle extends SequentialCommandGroup {
  Drive drive = Drive.getInstance();
  SuperStructure superStructure = SuperStructure.getInstance();

  public RightOutpostCycle() {
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
            .alongWith(
                superStructure.runOnce(
                    () -> superStructure.setIntakeMode(SuperStructure.IntakeMode.INTAKE))));
    addCommands(drive.followPPPath("RightNA-Right"));
    addCommands(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE, true).withTimeout(3.));
    addCommands(
        drive
            .followPPPath("Right-Outpost")
            .alongWith(
                superStructure.runOnce(
                    () -> superStructure.setIntakeMode(SuperStructure.IntakeMode.SHAKE)))
            .alongWith(new HybridShootCommand(Button.kAutoButton, ShootMode.SCORE, true)));
  }
}
