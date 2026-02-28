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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.StretcherConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.ManualShootFieldRelativeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.GyroIO;
import frc.robot.subsystems.Drive.GyroIOPigeon2;
import frc.robot.subsystems.Drive.GyroIOSim;
import frc.robot.subsystems.Drive.ModuleIO;
import frc.robot.subsystems.Drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.Drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Stretcher.StretcherSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Turret.TurretSubsystem;
import java.util.Set;
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
  // Subsystems
  public static final String limelightLeft = "limelight-left";
  public static final String limelightRight = "limelight-right";
  private final Drive drive;
  private final FeederSubsystem feeder = FeederSubsystem.getInstance();
  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final StretcherSubsystem stretcher = StretcherSubsystem.getInstance();
  private final TurretSubsystem turret = TurretSubsystem.getInstance();
  private final ClimberSubsystem climber = ClimberSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();
  // Simulated subsystems
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  public static final ImprovedCommandXboxController driverController =
      new ImprovedCommandXboxController(0);
  public static final ImprovedCommandXboxController operatorController =
      new ImprovedCommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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

    // configureButtonBindings();
    testBindings();
  }

  /**
   * ***** THE CONTROL LOGIC IS SUCH ******
   *
   * <p>DRIVER CONROLLER: B：RESET GYRO X: STOP WITH X RB: INTAKE LB: TRIGGER SHOOT MODE PovDown:
   * TOGGLE SHOOT MODE MANUAL/HYBRID PovUp: TOGGLE CONTROL MODE PASS/SCORE
   *
   * <p>OPERATOR CONTROLLER: RT: SHOOT RB: RESET SCORING UNIT HYBRID SHOOTING: LEFT STICK Y: HOOD
   * SCALAR RIGHT STICK X: TURRET SCALAR MANUAL SHOOTING (CURRENTLY UNUSED): LEFT STICK Y: HOOD
   * SCALAR RIGHT STICK X: TURRET SCALAR A: SET RPS 30 B: SET RPS 35 X: SET RPS 40 Y: SET RPS 45
   * MANUAL FIELD-RELATIVE SHOOTING: LEFT STICK: AIMING (TURRET SETPOINT IS CALCULATED BASED ON
   * ROBOT HEADING AND STICK ANGLE) RIGHT STICK Y: HOOD SCALAR A: SET RPS 30 B: SET RPS 35 X: SET
   * RPS 40 Y: SET RPS 45 LEFT BUMPER: INCREASE RPS BY 5 LEFT TRIGGER: DECREASE RPS BY 5
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        drive.run(
            () ->
                drive.driveFieldCentric(
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> -driverController.getRightX())));
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    driverController
        .rightBumper()
        .toggleOnTrue(
            Commands.defer(() -> superStructure.getIntakeCommand(), Set.of(stretcher, intake)));

    driverController
        .leftBumper()
        .whileTrue(
            Commands.defer(
                () -> superStructure.getShootCommand(Button.kRightTrigger, Button.kRightBumper),
                Set.of(feeder, hood, shooter, turret)));
    driverController
        .povDown()
        .onTrue(superStructure.runOnce(() -> superStructure.toggleControlMode()));
    driverController.povUp().onTrue(superStructure.runOnce(() -> superStructure.toggleShootMode()));
  }

  private void testBindings() {
    drive.setDefaultCommand(
        drive.run(
            () ->
                drive.driveFieldCentricWithMaxSpeed(
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> -driverController.getRightX(),
                    2.)));

    // driverController
    //     .a()
    //     .onTrue(new InstantCommand(() -> feeder.setTurntableRPS(2.5)))
    //     .onFalse(new InstantCommand(() -> feeder.setTurntableRPS(0.)));
    // driverController
    //     .a()
    //     .onTrue(new InstantCommand(() -> feeder.setFeedRPS(FeederConstants.DefaultFeedRPS)))
    //     .onFalse(new InstantCommand(() -> feeder.setFeedRPS(0.)));
    // driverController
    //     .a()
    //     .onTrue(
    //         new InstantCommand(() ->
    // stretcher.setPosition(StretcherConstants.ExtendedPosition)));
    // driverController
    //     .b()
    //     .onTrue(
    //         new InstantCommand(() ->
    // stretcher.setPosition(StretcherConstants.RetractedPosition)));
    // driverController
    //     .x()
    //     .onTrue(new InstantCommand(() -> intake.setRPS(IntakeConstants.IntakingRPS)))
    //     .onFalse(new InstantCommand(() -> intake.setRPS(0.)));
    driverController
        .rightBumper()
        .toggleOnTrue(
            Commands.defer(() -> superStructure.getIntakeCommand(), Set.of(stretcher, intake)));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                () -> superStructure.getClimbExtendCommand()));

    driverController
        .back()
        .onTrue(
            Commands.runOnce(
                () -> superStructure.getClimbRetractCommand()));
    
    operatorController
        .leftBumper()
        .whileTrue(
            Commands.defer(
                () ->
                    superStructure.getManualShootCommand(Button.kRightTrigger, Button.kRightBumper),
                Set.of(turret, shooter, hood, feeder)));
    // operatorController
    //     .leftBumper()
    //     .whileTrue(new ManualShootCommand(Button.kRightTrigger, Button.kRightBumper));
    // driverController.a().onTrue(new InstantCommand(()->hood.setPosition(44)));
    // driverController.b().onTrue(new InstantCommand(()->hood.setPosition(0.)));
    // driverController.a().onTrue(new InstantCommand(()->turret.setAutoSetpoint(-180.)));
    // driverController.b().onTrue(new InstantCommand(()->turret.setAutoSetpoint(40.)));
    // driverController.y().onTrue(new InstantCommand(()->turret.setAutoSetpoint(180.)));
    // driverController
    //     .a()
    //     .onTrue(new InstantCommand(() -> shooter.setRPS(20.)))
    //     .onFalse(new InstantCommand(() -> shooter.setRPS(0)));
    // driverController
    //     .y()
    //     .onTrue(new InstantCommand(() -> shooter.setRPS(45.)))
    //     .onFalse(new InstantCommand(() -> shooter.setRPS(0)));
    // driverController
    //     .x()
    //     .onTrue(new InstantCommand(() -> shooter.setRPS(80.)))
    //     .onFalse(new InstantCommand(() -> shooter.setRPS(0)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // Simulation methods
  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}