package frc.robot.util.turretAiming.mobileScoring;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.RobotContainer;
import frc.robot.util.turretAiming.constants.ArenaConstants;
import frc.robot.util.turretAiming.constants.TurretConstants;

public class AimTool {

    private static TurretInverseSolutionCalculator aimingSystem = new TurretInverseSolutionCalculator();
    public static double launchVelocity = 0.;
    public static double elevationAngle = 0.;
    public static double azimuthAngle = 0.;
    public static String status = "IDLE";
    public static double lastPitch =0.;
    public static double lastYaw =0.;
    public static double lastLaunchVel =0.;
    public static double lastTurretYawX =0.;
    public static double lastTurretYawY =0.;

    public void publishData(Pose2d robotPose) {
        if(RobotContainer.startAim){
            AimTool.calculateOnce(
                robotPose.getX() + TurretConstants.turretOffsetX,
                robotPose.getY() + TurretConstants.turretOffsetY,
                TurretConstants.turretHeight,
                RobotContainer.drive.getChassisSpeeds().vxMetersPerSecond,
                RobotContainer.drive.getChassisSpeeds().vyMetersPerSecond,
                ArenaConstants.kHub.getX(),
                ArenaConstants.kHub.getY(),
                ArenaConstants.kHub.getZ());
            Logger.recordOutput("AimingClac/Positions/Turret", new Pose3d(
                robotPose.getX() + TurretConstants.turretOffsetX,
                robotPose.getY() + TurretConstants.turretOffsetY,
                TurretConstants.turretHeight,
                new Rotation3d(
                0.,
                -Math.toRadians(AimTool.elevationAngle),
                Math.toRadians(AimTool.azimuthAngle)
                )));
                lastPitch =-Math.toRadians(AimTool.elevationAngle);
                lastYaw = Math.toRadians(AimTool.azimuthAngle);
                lastTurretYawX = Math.cos(Math.toRadians(AimTool.azimuthAngle));
                lastTurretYawY = Math.sin(Math.toRadians(AimTool.azimuthAngle));
                lastLaunchVel = AimTool.launchVelocity;
        }
        {
        Logger.recordOutput("AimingClac/Positions/Turret", new Pose3d(
            robotPose.getX() + TurretConstants.turretOffsetX,
            robotPose.getY() + TurretConstants.turretOffsetY,
            TurretConstants.turretHeight,
            new Rotation3d(
            0.,
            lastPitch,
            lastYaw
            )
            ));
        }
        Logger.recordOutput("AimingClac/Status", AimTool.status);
        Logger.recordOutput("AimingClac/LaunchVelocity", AimTool.launchVelocity);
        Logger.recordOutput("AimingClac/ElevationAngle", AimTool.elevationAngle);
        Logger.recordOutput("AimingClac/AzimuthAngle", AimTool.azimuthAngle);
    }

    /**
     * Calculate the ballistic solution from turret to target once (with XY chassis velocity compensation)
     */
    public static void calculateOnce(double turretX, double turretY, double turretZ,
                                     double turretVx, double turretVy,
                                     double targetX, double targetY, double targetZ) {

        aimingSystem.setTarget(targetX, targetY, targetZ);
        BallisticSolution solution = aimingSystem.calculateOptimalTrajectory(
                turretX, turretY, turretZ,
                turretVx, turretVy
        );

        if (solution != null 
            && !Double.isNaN(solution.launchVelocity) 
            && !Double.isNaN(solution.elevationAngle)) {

            launchVelocity = solution.launchVelocity;
            elevationAngle = solution.elevationAngle;
            azimuthAngle = solution.azimuthAngle;
            status = "OK";

        } else {
            if (aimingSystem.isTargetTooFar()) {
                status = "TOO_FAR";
            } else if (aimingSystem.isTargetTooClose()) {
                status = "TOO_CLOSE";
            } else {
                status = "IDLE"; // shouldn't happen
            }
        }
    }

    /* ------------------------------------------------T E S T----------------------------------------------- */
    // public static void main(String[] args) {
    //     Pose2d robotPose = new Pose2d(1.0, 1.0, null); // Example robot pose
    //     double turretX = robotPose.getX() + TurretConstants.turretOffsetX;
    //     double turretY = robotPose.getY() + TurretConstants.turretOffsetY;
    //     double turretZ = TurretConstants.turretHeight;
    
    //     // Example chassis velocity (m/s)
    //     double turretVx = 2.0;
    //     double turretVy = 0.5;
    
    //     double targetX = ArenaConstants.kTestScoringElement.getX();
    //     double targetY = ArenaConstants.kTestScoringElement.getY();
    //     double targetZ = ArenaConstants.kTestScoringElement.getZ();
    
    //     calculateOnce(turretX, turretY, turretZ, turretVx, turretVy, targetX, targetY, targetZ);
    
    //     System.out.printf("Status=%s, V=%.2f m/s, Elev=%.2f°, Azim=%.2f°%n",
    //             status, launchVelocity, elevationAngle, azimuthAngle);
    // }

}
