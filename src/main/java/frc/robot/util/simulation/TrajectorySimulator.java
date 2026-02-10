package frc.robot.util.simulation;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotContainer;
import frc.robot.util.turretAiming.constants.TurretConstants;
import frc.robot.util.turretAiming.mobileScoring.AimTool;

public class TrajectorySimulator {
    public void setTrajectory(Pose2d pose) {
        LinearVelocity launchVel = Units.MetersPerSecond.of(AimTool.lastLaunchVel + 0.5); //TODO: I DONT KNOW WHY THE PREVIOUS VALUE JUST DOESNT WORK (MASS??)
        Angle elevationAng = Units.Degrees.of(Math.toDegrees(-AimTool.lastPitch));
        SimulatedArena.getInstance()
            .addGamePieceProjectile(new RebuiltFuelOnFly(
                pose.getTranslation(),
                new Translation2d(
                    TurretConstants.turretOffsetX,
                    TurretConstants.turretOffsetY),
                RobotContainer.drive.getChassisSpeeds(),
                new Rotation2d(
                    AimTool.lastTurretYawX,
                    AimTool.lastTurretYawY
                  ),
                Meters.of(TurretConstants.turretHeight),
                launchVel,
                elevationAng)
            .withProjectileTrajectoryDisplayCallBack(
                (poses) -> Logger.recordOutput("shotsTrajectory", poses.toArray(Pose3d[]::new)))
            .enableBecomesGamePieceOnFieldAfterTouchGround());
    }
}
