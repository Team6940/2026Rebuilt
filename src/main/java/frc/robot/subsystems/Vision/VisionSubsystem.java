// Copyright 2021-2025 FRC 6328 / team extensions
//
// Fuses multiple independent vision pose measurements into {@link
// edu.wpi.first.math.estimator.SwerveDrivePoseEstimator} using per-measurement standard deviations
// (no manual pose averaging).

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PoseEstimatorConstants;
import frc.robot.Constants.VisionFusion;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.Drive;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Collects Limelight MegaTag2 (σ from NT {@code stddevs}; see {@link #readMegaTag2StdDevs(String)})
 * and PhotonVision estimates. Photon translation σ uses {@link PoseEstimatorConstants#tAtoDev} from
 * mean target area fraction ({@code area}/100).
 */
public class VisionSubsystem extends SubsystemBase {

  private final Drive drive;
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator photonPoseEstimator;
  public static VisionSubsystem m_instance;

  private final NetworkTableEntry llLeftSnapshot =
      NetworkTableInstance.getDefault().getTable(RobotContainer.limelightLeft).getEntry("snapshot");
  private final NetworkTableEntry llRightSnapshot =
      NetworkTableInstance.getDefault()
          .getTable(RobotContainer.limelightRight)
          .getEntry("snapshot");


  public static VisionSubsystem getInstance(Drive drive) {
    return m_instance == null ? m_instance = new VisionSubsystem(drive) : m_instance;
  }

  public VisionSubsystem(Drive drive) {
    this.drive = drive;
    this.photonCamera = new PhotonCamera(RobotContainer.photonCameraName);
    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            Constants.FieldConstants.getAprilTagFieldLayout(), VisionFusion.kRobotToCamera);
  }

  @Override
  public void periodic() {
    double fpgaNow = Timer.getFPGATimestamp();

    llLeftSnapshot.setNumber(0);
    llRightSnapshot.setNumber(0);

    processLimelight(RobotContainer.limelightLeft, "Left", fpgaNow);
    processLimelight(RobotContainer.limelightRight, "Right", fpgaNow);
    processPhoton(fpgaNow);
  }

  private void processLimelight(String limelightName, String logSide, double fpgaNow) {
    drive.applyLimelightGyroForMegaTag2(limelightName);

    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    logLimelightPoseEstimate(logSide, mt2);

    if (mt2 == null) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Connected", false);
      return;
    }

    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Connected", true);

    if (mt2.tagCount > 0) {
      NetworkTableEntry snapshotEntry =
          limelightName.equals(RobotContainer.limelightLeft) ? llLeftSnapshot : llRightSnapshot;
      snapshotEntry.setNumber(1);
    }

    ChassisSpeeds speeds = drive.getChassisSpeeds();

    double tagDistRobotM = limelightTagDistanceToRobotMeters(mt2);

    if (shouldReject(
        mt2.avgTagArea,
        mt2.tagCount,
        mt2.timestampSeconds,
        tagDistRobotM,
        fpgaNow,
        speeds)) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", false);
      return;
    }

    Optional<LlMegaTag2StdDevs> stdOpt = readMegaTag2StdDevs(limelightName);
    if (stdOpt.isEmpty()) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevsValid", false);
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", false);
      return;
    }
    LlMegaTag2StdDevs std = stdOpt.get();
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevsValid", true);
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevX", std.sigmaXMeters());
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevY", std.sigmaYMeters());
    Logger.recordOutput(
        "VisionFusion/Limelight/" + limelightName + "/StdDevYawRad", std.sigmaYawRadians());

    drive.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds,
        VecBuilder.fill(std.sigmaXMeters(), std.sigmaYMeters(), std.sigmaYawRadians()));
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", true);
  }

  private void processPhoton(double fpgaNow) {
    photonPoseEstimator.addHeadingData(fpgaNow, drive.getPose().getRotation());

    List<PhotonPipelineResult> unread = photonCamera.getAllUnreadResults();
    PhotonPipelineResult result =
        unread.stream()
            .filter(PhotonPipelineResult::hasTargets)
            .max(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds))
            .orElse(null);
    if (result == null) {
      logPhotonPoseEstimate(Optional.empty(), Optional.empty());
      return;
    }

    Optional<EstimatedRobotPose> multiOpt = photonPoseEstimator.estimateCoprocMultiTagPose(result);
    Optional<EstimatedRobotPose> pnpOpt =
        multiOpt.isPresent()
            ? Optional.empty()
            : photonPoseEstimator.estimatePnpDistanceTrigSolvePose(result);
    logPhotonPoseEstimate(multiOpt, pnpOpt);

    if (multiOpt.isPresent()) {
      EstimatedRobotPose est = multiOpt.get();
      Pose2d pose = est.estimatedPose.toPose2d();
      List<PhotonTrackedTarget> targets = est.targetsUsed;
      int tagCount = Math.max(1, targets.size());
      double yawRad = maxAbsPhotonYawRad(targets);
      double taMean = photonAvgAreaFraction(targets);


      double tagDistM = photonAvgCameraToTagDistanceMeters(targets);

      if (shouldReject(
          taMean,
          tagCount,
          est.timestampSeconds,
          tagDistM,
          fpgaNow,
          drive.getChassisSpeeds())) {
        return;
      }

      double xyStd = PoseEstimatorConstants.tAtoDev.get(taMean);
      drive.addVisionMeasurement(
          pose,
          est.timestampSeconds,
          VecBuilder.fill(xyStd, xyStd, VisionFusion.PHOTON_THETA_STDDEV_RADIANS));
      return;
    }

    if (pnpOpt.isEmpty()) {
      return;
    }

    EstimatedRobotPose est = pnpOpt.get();
    Pose2d pose = est.estimatedPose.toPose2d();
    PhotonTrackedTarget best = result.getBestTarget();
    if (best == null) {
      return;
    }

    double taFrac = best.area / 100.0;
    double yawRad = Units.degreesToRadians(Math.abs(best.yaw));

    double tagDistM = photonCameraToTagDistanceMeters(best);

    if (shouldReject(
        taFrac,
        1,
        est.timestampSeconds,
        tagDistM,
        fpgaNow,
        drive.getChassisSpeeds())) {
      return;
    }

    double xyStd = PoseEstimatorConstants.tAtoDev.get(taFrac);
    drive.addVisionMeasurement(
        pose,
        est.timestampSeconds,
        VecBuilder.fill(xyStd, xyStd, VisionFusion.PHOTON_THETA_STDDEV_RADIANS));
  }

  /**
   * Logs MegaTag2 {@link LimelightHelpers.PoseEstimate} data for AdvantageScope (pose + metadata).
   */
  private static void logLimelightPoseEstimate(String side, LimelightHelpers.PoseEstimate mt2) {
    String base = "VisionFusion/PoseEstimates/Limelight" + side;
    if (mt2 == null) {
      Logger.recordOutput(base, new Pose2d());
      Logger.recordOutput(base + "/HasData", false);
      return;
    }
    Logger.recordOutput(base, mt2.pose);
    Logger.recordOutput(base + "/HasData", mt2.tagCount > 0);
    Logger.recordOutput(base + "/TimestampSeconds", mt2.timestampSeconds);
    Logger.recordOutput(base + "/Latency", mt2.latency);
    Logger.recordOutput(base + "/TagCount", mt2.tagCount);
    Logger.recordOutput(base + "/TagSpan", mt2.tagSpan);
    Logger.recordOutput(base + "/AvgTagDist", mt2.avgTagDist);
    Logger.recordOutput(base + "/AvgTagArea", mt2.avgTagArea);
    Logger.recordOutput(base + "/MegaTag2", mt2.isMegaTag2);
  }

  /**
   * Limelight MegaTag2 σ published on NT {@code stddevs}: {@code [MT1…, MT2x, MT2y, MT2z, MT2roll,
   * MT2pitch, MT2yaw]}.
   *
   * <p>Translation σ in meters; rotational σ in degrees — yaw converted to radians for WPILib
   * fusion.
   *
   * @see <a href="https://docs.limelightvision.io/zh/docs/docs-limelight/apis/complete-networktables-api">Limelight
   *     NetworkTables API</a>
   */
  private static Optional<LlMegaTag2StdDevs> readMegaTag2StdDevs(String limelightName) {
    double[] arr =
        NetworkTableInstance.getDefault()
            .getTable(sanitizeLimelightNetworkTableName(limelightName))
            .getEntry("stddevs")
            .getDoubleArray(new double[0]);
    final int mt2Base = 6;
    if (arr.length < mt2Base + 6) {
      return Optional.empty();
    }
    double sigmaX = arr[mt2Base];
    double sigmaY = arr[mt2Base + 1];
    double sigmaYawDeg = arr[mt2Base + 5];
    if (!Double.isFinite(sigmaX)
        || !Double.isFinite(sigmaY)
        || !Double.isFinite(sigmaYawDeg)) {
      return Optional.empty();
    }
    if (sigmaX <= 0.0 || sigmaY <= 0.0 || sigmaYawDeg <= 0.0) {
      return Optional.empty();
    }
    return Optional.of(
        new LlMegaTag2StdDevs(sigmaX, sigmaY, Units.degreesToRadians(sigmaYawDeg)));
  }

  /** Mirrors Limelight-generated {@code sanitizeName} for NetworkTable root. */
  private static String sanitizeLimelightNetworkTableName(String name) {
    if (name == null || name.isEmpty()) {
      return "limelight";
    }
    return name;
  }

  private record LlMegaTag2StdDevs(
      double sigmaXMeters, double sigmaYMeters, double sigmaYawRadians) {}

  /**
   * Logs Photon estimated pose (multi-tag preferred, else PNP distance trig) for AdvantageScope.
   */
  private static void logPhotonPoseEstimate(
      Optional<EstimatedRobotPose> multiOpt, Optional<EstimatedRobotPose> pnpOpt) {
    String base = "VisionFusion/PoseEstimates/Photon";
    if (multiOpt.isPresent()) {
      EstimatedRobotPose est = multiOpt.get();
      Logger.recordOutput(base, est.estimatedPose.toPose2d());
      Logger.recordOutput(base + "/HasData", true);
      Logger.recordOutput(base + "/TimestampSeconds", est.timestampSeconds);
      Logger.recordOutput(base + "/Strategy", "MULTI_TAG_PNP");
      Logger.recordOutput(base + "/TagCount", est.targetsUsed.size());
      return;
    }
    if (pnpOpt.isPresent()) {
      EstimatedRobotPose est = pnpOpt.get();
      Logger.recordOutput(base, est.estimatedPose.toPose2d());
      Logger.recordOutput(base + "/HasData", true);
      Logger.recordOutput(base + "/TimestampSeconds", est.timestampSeconds);
      Logger.recordOutput(base + "/Strategy", "SINGLE_TAG_PNP_DISTANCE_TRIG");
      Logger.recordOutput(base + "/TagCount", est.targetsUsed.size());
      return;
    }
    Logger.recordOutput(base, new Pose2d());
    Logger.recordOutput(base + "/HasData", false);
    Logger.recordOutput(base + "/Strategy", "none");
  }

  private static double photonAvgAreaFraction(List<PhotonTrackedTarget> targets) {
    if (targets.isEmpty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (PhotonTrackedTarget t : targets) {
      sum += t.area / 100.0;
    }
    return sum / targets.size();
  }

  /**
   * Shared rejection gates for all vision sources. {@code ta} is Limelight-style fractional area
   * (0–1) or Photon fractional area ({@code area}/100).
   *
   * <p>{@code tagDistanceMeters}: Limelight uses mean {@link LimelightHelpers.RawFiducial#distToRobot}
   * when fiducials exist; otherwise {@link LimelightHelpers.PoseEstimate#avgTagDist}. Photon uses mean
   * camera–tag range from {@code getBestCameraToTarget()} (meters).
   */
  static boolean shouldReject(
      double ta,
      int tagCount,
      double measurementTimestampSeconds,
      double tagDistanceMeters,
      double fpgaNow,
      ChassisSpeeds chassisSpeeds) {

    if (ta < VisionFusion.REJECT_MIN_TA) {
      return true;
    }
    if (tagCount <= 0) {
      return true;
    }
    if (tagDistanceMeters > VisionFusion.REJECT_MAX_DISTANCE_METERS) {
      return true;
    }
    if (fpgaNow - measurementTimestampSeconds > VisionFusion.REJECT_STALE_SECONDS) {
      return true;
    }
    if (VisionFusion.REJECT_ON_HIGH_OMEGA
        && Math.abs(chassisSpeeds.omegaRadiansPerSecond)
            > VisionFusion.REJECT_MAX_OMEGA_RAD_PER_SEC) {
      return true;
    }
    return false;
  }

  /**
   * Tag–robot distance from Limelight: arithmetic mean {@link LimelightHelpers.RawFiducial#distToRobot};
   * if fiducials are missing uses {@link LimelightHelpers.PoseEstimate#avgTagDist} (meters).
   */
  private static double limelightTagDistanceToRobotMeters(LimelightHelpers.PoseEstimate mt2) {
    if (mt2.rawFiducials != null && mt2.rawFiducials.length > 0) {
      double sum = 0.0;
      for (LimelightHelpers.RawFiducial f : mt2.rawFiducials) {
        sum += f.distToRobot;
      }
      return sum / mt2.rawFiducials.length;
    }
    return mt2.avgTagDist;
  }

  /** Mean camera–tag range (solve translation norm, meters) over visible targets. */
  private static double photonAvgCameraToTagDistanceMeters(List<PhotonTrackedTarget> targets) {
    if (targets.isEmpty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (PhotonTrackedTarget t : targets) {
      sum += photonCameraToTagDistanceMeters(t);
    }
    return sum / targets.size();
  }

  private static double photonCameraToTagDistanceMeters(PhotonTrackedTarget t) {
    return t.getBestCameraToTarget().getTranslation().getNorm();
  }


  private static double maxAbsPhotonYawRad(List<PhotonTrackedTarget> targets) {
    double maxDeg = 0.0;
    for (PhotonTrackedTarget t : targets) {
      maxDeg = Math.max(maxDeg, Math.abs(t.yaw));
    }
    return Units.degreesToRadians(maxDeg);
  }
}
