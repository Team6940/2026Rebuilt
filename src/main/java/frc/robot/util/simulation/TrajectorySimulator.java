package frc.robot.util.simulation;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class TrajectorySimulator {
    public void setTrajectory(Pose2d pose) {
        ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        HoodSubsystem hood = HoodSubsystem.getInstance();
        TurretSubsystem turret = TurretSubsystem.getInstance();

        double shooterRps = shooter.getShooterRPS();
        double launchSpeedMps = shooterRps * 2.0 * Math.PI * ShooterConstants.ShooterWheelRadiusMeters;
        LinearVelocity launchVel = Units.MetersPerSecond.of(launchSpeedMps);

        double hoodDegs = hood.getCurrentPositionDegs();
        Angle elevationAng = Units.Degrees.of(hoodDegs);
        Rotation2d turretRotation = Rotation2d.fromDegrees(turret.getCurrentPositionDegs());
        SimulatedArena.getInstance()
            .addGamePieceProjectile(new RebuiltFuelOnFly(
                pose.getTranslation(),
                TurretConstants.TURRET_OFFSET,
                RobotContainer.drive.getChassisSpeeds(),
                turretRotation,
                Meters.of(TurretConstants.TurretHeightMeters),
                launchVel,
                elevationAng)
            .withProjectileTrajectoryDisplayCallBack(
                (poses) -> Logger.recordOutput("shotsTrajectory", poses.toArray(Pose3d[]::new)))
            .enableBecomesGamePieceOnFieldAfterTouchGround());
    }
}
