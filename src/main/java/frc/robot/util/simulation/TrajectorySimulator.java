package frc.robot.util.simulation;

import static edu.wpi.first.units.Units.Meters;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;

/**
 * How to vizualize trajectories in AScope:
 * 1. Run a sim and TRIGGER a trajectory to be emitted FIRST
 * 2. In AScope, add a "Pose3d Array" data source with the key "shotsTrajectory"
 */
public class TrajectorySimulator {
    private static final int LOOPS_PER_SHOT = 1;
    private int loopCounter = 0;

    public void setTrajectory(Pose2d pose, double hoodDegs, double shooterRps, Rotation2d turretRotation) {
        // Rate limiting: Only emit a trajectory every N loops to avoid spawning too many projectiles
        loopCounter++;
        if (loopCounter < LOOPS_PER_SHOT) {
            return; // Skip this call
        }
        loopCounter = 0; // Reset counter

        // Lookup ball speed from RPS using interpolation table
        double launchSpeedMps = ShooterRpsToMpsInterpolationTable.PRESET_1.get(shooterRps);
        
        // Lookup launch angle from hood angle using interpolation table
        double launchAngleDegrees = HoodAngleToLaunchAngleInterpolationTable.TABLE.get(hoodDegs);

        SimulatedArena.getInstance()
            .addGamePieceProjectile(new RebuiltFuelOnFly(
                pose.getTranslation(),
                TurretConstants.TURRET_OFFSET,
                RobotContainer.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(), //TODO
                turretRotation,
                Meters.of(TurretConstants.TurretHeightMeters),
                Units.MetersPerSecond.of(launchSpeedMps),
                Units.Degrees.of(launchAngleDegrees))
            .withProjectileTrajectoryDisplayCallBack(
                (poses) -> Logger.recordOutput("FieldSimulation/ShotsTrajectory", poses.toArray(Pose3d[]::new)))
            .enableBecomesGamePieceOnFieldAfterTouchGround());

        // Record the incoming parameters so we can verify they're changing in SIM
        Logger.recordOutput("FieldSimulation/ShotPose2d", pose);
        Logger.recordOutput("FieldSimulation/ShotHoodDegs", hoodDegs);
        Logger.recordOutput("FieldSimulation/ShotLaunchAngleDegs", launchAngleDegrees);
        Logger.recordOutput("FieldSimulation/ShotRPS", shooterRps);
        Logger.recordOutput("FieldSimulation/ShotMPS", launchSpeedMps);
        Logger.recordOutput("FieldSimulation/ShotTurretRotationDegs", turretRotation.getDegrees());
    }
}
