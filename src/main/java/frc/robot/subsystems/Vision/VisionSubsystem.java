// Copyright 2021-2025 FRC 6328 / team extensions
//
// Fuses multiple independent vision pose measurements into {@link
// edu.wpi.first.math.estimator.SwerveDrivePoseEstimator} using per-measurement standard deviations
// (no manual pose averaging).

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionFusion;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Feeder.FeederSubsystem;

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
 * Collects Limelight MegaTag2 and PhotonVision (multi-tag with single-tag fallback) estimates and
 * feeds them independently into the drive pose estimator with dynamically computed standard
 * deviations.
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
    Pose2d refPose = drive.getPose();
    double fpgaNow = Timer.getFPGATimestamp();

    llLeftSnapshot.setNumber(0);
    llRightSnapshot.setNumber(0);

    processLimelight(RobotContainer.limelightLeft, "Left", refPose, fpgaNow);
    processLimelight(RobotContainer.limelightRight, "Right", refPose, fpgaNow);
    processPhoton(refPose, fpgaNow);
  }

  private void processLimelight(
      String limelightName, String logSide, Pose2d refPose, double fpgaNow) {
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

    double maxAmb = maxLimelightAmbiguity(mt2);
    double yawRad = maxAbsLimelightTxRad(mt2);
    ChassisSpeeds speeds = drive.getChassisSpeeds();

    if (shouldReject(
        maxAmb,
        mt2.avgTagArea,
        mt2.tagCount,
        mt2.pose,
        mt2.timestampSeconds,
        refPose,
        fpgaNow,
        speeds)) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", false);
      return;
    }

    double xyStd = limelightMegaTag2XyStdDev(mt2, maxAmb);
    double thetaStd = limelightMegaTag2ThetaStdDev(mt2.avgTagArea, maxAmb);
    xyStd = applyGlobalYawPenaltiesAndClamp(xyStd, yawRad);

    drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, buildStdDevs(xyStd, thetaStd));
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", true);
  }

  private void processPhoton(Pose2d refPose, double fpgaNow) {
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
      double maxAmb = maxPhotonAmbiguity(targets);
      double minTaFrac = minPhotonAreaFraction(targets);

      if (shouldReject(
          maxAmb,
          minTaFrac,
          tagCount,
          pose,
          est.timestampSeconds,
          refPose,
          fpgaNow,
          drive.getChassisSpeeds())) {
        return;
      }

      double xyStd = photonMultiTagXyStdDev(tagCount);
      double thetaStd = Units.degreesToRadians(5.0);
      xyStd = applyGlobalYawPenaltiesAndClamp(xyStd, yawRad);
      drive.addVisionMeasurement(pose, est.timestampSeconds, buildStdDevs(xyStd, thetaStd));
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
    double amb = best.poseAmbiguity >= 0 ? best.poseAmbiguity : 0.0;

    if (shouldReject(
        amb, taFrac, 1, pose, est.timestampSeconds, refPose, fpgaNow, drive.getChassisSpeeds())) {
      return;
    }

    double xyStd = photonSingleTagXyStdDev(taFrac);
    double thetaStd = Units.degreesToRadians(15.0);
    xyStd = applyGlobalYawPenaltiesAndClamp(xyStd, yawRad);
    drive.addVisionMeasurement(pose, est.timestampSeconds, buildStdDevs(xyStd, thetaStd));
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

  /** MegaTag2 horizontal standard deviation before global yaw scaling. */
  static double limelightMegaTag2XyStdDev(LimelightHelpers.PoseEstimate mt2, double maxAmbiguity) {
    double ta = Math.max(mt2.avgTagArea, 1e-6);
    double xy = VisionFusion.LIMELIGHT_XY_K / Math.sqrt(ta);
    xy *= (1.0 + 2.5 * maxAmbiguity);
    if (mt2.tagCount == 1) {
      xy *= 1.7;
    }
    return xy;
  }

  static double limelightMegaTag2ThetaStdDev(double avgTagArea, double maxAmbiguity) {
    if (maxAmbiguity < 0.1 && avgTagArea > 0.05) {
      return Units.degreesToRadians(8.0);
    }
    return Units.degreesToRadians(20.0);
  }

  static double photonMultiTagXyStdDev(int tagCount) {
    int n = Math.max(1, tagCount);
    return 0.05 / Math.sqrt(n);
  }

  static double photonSingleTagXyStdDev(double taFrac) {
    double ta = Math.max(taFrac, 1e-6);
    return VisionFusion.PHOTON_SINGLE_XY_K / Math.sqrt(ta);
  }

  /**
   * Applies {@code xyStd /= cos(|yaw|)} with a minimum cosine divisor, then clamps to field {@link
   * VisionFusion#XY_STDDEV_MIN} … {@link VisionFusion#XY_STDDEV_MAX}.
   */
  static double applyGlobalYawPenaltiesAndClamp(double xyStd, double yawRad) {
    double denom = Math.max(Math.cos(Math.abs(yawRad)), VisionFusion.MIN_COS_YAW);
    xyStd /= denom;
    return MathUtil.clamp(xyStd, VisionFusion.XY_STDDEV_MIN, VisionFusion.XY_STDDEV_MAX);
  }

  static Matrix<N3, N1> buildStdDevs(double xyStd, double thetaStd) {
    return VecBuilder.fill(xyStd, xyStd, thetaStd);
  }

  /**
   * Shared rejection gates for all vision sources. {@code ta} is Limelight-style fractional area
   * (0–1) or Photon fractional area ({@code area}/100).
   */
  static boolean shouldReject(
      double maxAmbiguity,
      double ta,
      int tagCount,
      Pose2d visionPose,
      double measurementTimestampSeconds,
      Pose2d currentEstimate,
      double fpgaNow,
      ChassisSpeeds chassisSpeeds) {

    if (maxAmbiguity > VisionFusion.REJECT_MAX_AMBIGUITY) {
      return true;
    }
    if (ta < VisionFusion.REJECT_MIN_TA) {
      return true;
    }
    if (tagCount <= 0) {
      return true;
    }
    double dist = visionPose.getTranslation().getDistance(currentEstimate.getTranslation());
    if (dist > VisionFusion.REJECT_MAX_POSE_ERROR_METERS) {
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

  private static double maxLimelightAmbiguity(LimelightHelpers.PoseEstimate estimate) {
    if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
      return 0.0;
    }
    double max = 0.0;
    for (LimelightHelpers.RawFiducial f : estimate.rawFiducials) {
      max = Math.max(max, f.ambiguity);
    }
    return max;
  }

  /** Horizontal angle magnitude (rad) for yaw penalty: largest |txnc| among detected tags. */
  private static double maxAbsLimelightTxRad(LimelightHelpers.PoseEstimate estimate) {
    if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
      return 0.0;
    }
    double maxDeg = 0.0;
    for (LimelightHelpers.RawFiducial f : estimate.rawFiducials) {
      maxDeg = Math.max(maxDeg, Math.abs(f.txnc));
    }
    return Units.degreesToRadians(maxDeg);
  }

  private static double maxPhotonAmbiguity(List<PhotonTrackedTarget> targets) {
    double max = 0.0;
    for (PhotonTrackedTarget t : targets) {
      if (t.poseAmbiguity >= 0 && !Double.isNaN(t.poseAmbiguity)) {
        max = Math.max(max, t.poseAmbiguity);
      }
    }
    return max;
  }

  private static double minPhotonAreaFraction(List<PhotonTrackedTarget> targets) {
    double min = 1.0;
    for (PhotonTrackedTarget t : targets) {
      min = Math.min(min, t.area / 100.0);
    }
    return min;
  }

  private static double maxAbsPhotonYawRad(List<PhotonTrackedTarget> targets) {
    double maxDeg = 0.0;
    for (PhotonTrackedTarget t : targets) {
      maxDeg = Math.max(maxDeg, Math.abs(t.yaw));
    }
    return Units.degreesToRadians(maxDeg);
  }
}
