// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.controller.ImprovedCommandXboxController;
import frc.robot.subsystems.controller.KeyboardController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.util.simulation.TrajectorySimulator;
import frc.robot.util.simulation.bumpPhysicsEnhance.BumpConstants;
import frc.robot.util.simulation.bumpPhysicsEnhance.BumpPhysicsUtil;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final double DEADBAND = 0.05; // TODO CHANGE JOYSTICK DEADBAND HERE

  // Subsystems
  public static Drive drive;
  public static SwerveDriveSimulation driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d());
  // Controller
  public static final ImprovedCommandXboxController driveController = new ImprovedCommandXboxController(0);
  public static final KeyboardController keyboardController = new KeyboardController();
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  //custom
  public static boolean startAim = false;
  public TrajectorySimulator trajectorySetter = new TrajectorySimulator();
  private BumpPhysicsUtil.BumpState bump =
    BumpPhysicsUtil.BumpState.flat();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    configureKeyBoardBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        drive.run(
            () ->
                drive.driveFieldCentric(
                    () -> -driveController.getLeftY(),
                    () -> -driveController.getLeftX(),
                    () -> -driveController.getRightX())));

    // Lock to 0° when A button is held
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }
  
  private void configureKeyBoardBindings() {
    // double vx = 0, vy = 0, vz = 0;
    keyboardController.up().toggleOnTrue(
        drive.run(
            () ->
                drive.driveFieldCentric(
                    () -> 0.6,
                    () -> 0.0,
                    () -> 0.0)));
    keyboardController.down().toggleOnTrue(
        drive.run(
            () ->
                drive.driveFieldCentric(
                    () -> -0.6,
                    () -> 0.0,
                    () -> 0.0)));
    keyboardController.left().toggleOnTrue(
        drive.run(
            () ->
                drive.driveFieldCentric(
                    () -> 0.0,
                    () -> 0.6,
                    () -> 0.0)));
    keyboardController.right().toggleOnTrue(
        drive.run(
            () ->
                drive.driveFieldCentric(
                    () -> 0.0,
                    () -> -0.6,
                    () -> 0.0)));
    keyboardController.q().toggleOnTrue(
        drive.run(
            () ->
                drive.driveFieldCentric(
                    () -> 0.0,
                    () -> 0.0,
                    () -> Math.toRadians(60.0))));
    keyboardController.e().toggleOnTrue(
        drive.run(
            () ->
                drive.driveFieldCentric(
                    () -> 0.0,
                    () -> 0.0,
                    () -> Math.toRadians(-60.0))));
    keyboardController.z()
        .onTrue(Commands.runOnce(() -> startAim = true));
    keyboardController.v()
        .onTrue(Commands.runOnce(() -> startAim = false));
    keyboardController.i()
        .onTrue(Commands.runOnce(() -> trajectorySetter.setTrajectory(Robot.robotPose)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

public void updateSimulation() {
  if (Constants.currentMode != Constants.Mode.SIM) return;

  SimulatedArena.getInstance().simulationPeriodic();

  Pose2d pose = driveSimulation.getSimulatedDriveTrainPose();

  // === 核心：纯几何解算 ===
    bump = BumpPhysicsUtil.solve(pose);

Logger.recordOutput(
    "FieldSimulation/RobotPosition",
    new Pose3d(
        pose.getX(),
        pose.getY(),
        bump.z,
        bump.rotation.plus(
            new Rotation3d(
                0, 0,
                pose.getRotation().getRadians()))));

  Logger.recordOutput(
      "FieldSimulation/Fuel",
      SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));

  for (Pose2d b : BumpConstants.BUMP_CENTERS) {
    Logger.recordOutput(
        "FieldSimulation/BumpAt"
            + String.format("(%.2f,%.2f)", b.getX(), b.getY()),
        new Pose3d(
            b.getX(),
            b.getY(),
            BumpConstants.BUMP_HEIGHT / 2.0,
            new Rotation3d(
                0.0,
                0.0,
                b.getRotation().getRadians())));
  }
}
}
