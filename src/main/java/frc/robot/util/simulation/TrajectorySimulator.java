package frc.robot.util.simulation;

import static edu.wpi.first.units.Units.Meters;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;

/**
 * How to vizualize trajectories in AScope:
 * 1. Run a sim and TRIGGER a trajectory to be emitted FIRST
 * 2. In AScope, add a "Pose3d Array" data source with the key "shotsTrajectory"
 */
public class TrajectorySimulator {
    public void setTrajectory(Pose2d pose,double hoodDegs, double shooterRps, Rotation2d turretRotation) {
        // Record the incoming parameters so we can verify they're changing in SIM
        Logger.recordOutput("FieldSimulation/ShotPose2d", pose);
        Logger.recordOutput("FieldSimulation/ShotHoodDegs", hoodDegs);
        Logger.recordOutput("FieldSimulation/ShotRPS", shooterRps);
        Logger.recordOutput("FieldSimulation/ShotTurretRotationDegs", turretRotation.getDegrees());

        double launchSpeedMps = shooterRps * 2.0 * Math.PI * ShooterConstants.ShooterWheelRadiusMeters;
        // Rotation2d turretRotation = Rotation2d.fromDegrees(turretDegs);//TODO check if this is correct

        SimulatedArena.getInstance()
            .addGamePieceProjectile(new RebuiltFuelOnFly(
                pose.getTranslation(),
                TurretConstants.TURRET_OFFSET,
                RobotContainer.drive.getChassisSpeeds(), //TODO
                turretRotation,
                Meters.of(TurretConstants.TurretHeightMeters),
                Units.MetersPerSecond.of(launchSpeedMps),
                Units.Degrees.of(hoodDegs))
            .withProjectileTrajectoryDisplayCallBack(
                (poses) -> Logger.recordOutput("FieldSimulation/ShotsTrajectory", poses.toArray(Pose3d[]::new)))
            .enableBecomesGamePieceOnFieldAfterTouchGround());
    }
}
