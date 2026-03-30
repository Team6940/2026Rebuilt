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
    // Trajectory emission rate control: 7 projectiles per second = ~143ms between shots
    // At 50Hz loop rate: 50 loops/sec / 7 shots/sec = ~7 loops per shot
    private static final int LOOPS_PER_SHOT = 7; // Emit every 7 loops (50Hz / 7 ≈ 7 shots/sec)
    private int loopCounter = 0;

    public void setTrajectory(Pose2d pose, double hoodDegs, double shooterRps, Rotation2d turretRotation) {
        // Rate limiting: Only emit a trajectory every N loops to avoid spawning too many projectiles
        loopCounter++;
        if (loopCounter < LOOPS_PER_SHOT) {
            return; // Skip this call
        }
        loopCounter = 0; // Reset counter

        // Record the incoming parameters so we can verify they're changing in SIM
        Logger.recordOutput("FieldSimulation/ShotPose2d", pose);
        Logger.recordOutput("FieldSimulation/ShotHoodDegs", hoodDegs);
        Logger.recordOutput("FieldSimulation/ShotRPS", shooterRps);
        Logger.recordOutput("FieldSimulation/ShotTurretRotationDegs", turretRotation.getDegrees());

        //  Physical conversion: Flywheel speed → Ball speed
        //  Using ShooterPhysicsConverter to consider the following factors:
        //  1. Flywheel surface speed: v_contact = ω × r
        //  2. Friction coefficient: the strength of friction between the ball and the
        //  3. Slip loss: the velocity reduction due to slipping between the ball and the flywheel
        //  4. Compression factor: the energy loss due to contact compression
        double launchSpeedMps = ShooterPhysicsConverter.rpsToMps(shooterRps);

        SimulatedArena.getInstance()
            .addGamePieceProjectile(new RebuiltFuelOnFly(
                pose.getTranslation(),
                TurretConstants.TURRET_OFFSET,
                RobotContainer.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(), //TODO
                turretRotation,
                Meters.of(TurretConstants.TurretHeightMeters),
                Units.MetersPerSecond.of(launchSpeedMps),
                Units.Degrees.of(hoodDegs))
            .withProjectileTrajectoryDisplayCallBack(
                (poses) -> Logger.recordOutput("FieldSimulation/ShotsTrajectory", poses.toArray(Pose3d[]::new)))
            .enableBecomesGamePieceOnFieldAfterTouchGround());
    }
}
