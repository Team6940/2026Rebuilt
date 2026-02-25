package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.SuperStructure.ShootMode;
import frc.robot.subsystems.Turret.TurretSubsystem;

public final class ManualShotRecommender {
  private static final double LEAD_YAW_COMPENSATION_INDEX = 0.1;
  private static final double[] MANUAL_RPS_PRESETS = {
    ShooterConstants.ManualRpsA,
    ShooterConstants.ManualRpsB,
    ShooterConstants.ManualRpsX,
    ShooterConstants.ManualRpsY
  };

  private static final String KEY_TURRET = "ManualShot/RecommendedTurretDegs";
  private static final String KEY_HOOD = "ManualShot/RecommendedHoodDegs";
  private static final String KEY_RPS = "ManualShot/RecommendedShooterRps";

  /**
   * Updates the SmartDashboard with recommended turret angle, hood angle, and shooter RPS 
   * for a manual shot, based on the current state of the selected shoot mode.
   * @param shootMode the current shoot mode (SCORE, PASS, or FREE) which determines the target and compensation used
   * If shootMode is FREE or if drive/turret is null, NaN values will be published to indicate no recommendation
   * The recommended turret angle is calculated to aim at the target (hub or tower)
   */

  public static void updateSmartDashboard(
      Drive drive, TurretSubsystem turret, ShootMode shootMode) {
    if (drive == null || turret == null || shootMode == null) {
      return;
    }

    if (shootMode == ShootMode.FREE) {
      publishNaN();
      return;
    }

    double hoodDegs;
    double idealRps;
    double tangentialVelocity;
    Rotation2d fieldTargetAngle;

    if (shootMode == ShootMode.SCORE) {
      double distanceMeters = drive.getDistanceToAllianceHub();
      Translation2d relativeSpeeds = drive.getHubRelativeChassisSpeeds();
      double radialVelocity = relativeSpeeds.getX();
      tangentialVelocity = relativeSpeeds.getY();

      idealRps = ProjectileCalculator.estimateMotionShotRps(distanceMeters, radialVelocity);
      hoodDegs = ProjectileCalculator.estimateMotionShotHoodAngle(distanceMeters, radialVelocity);
      double leadYawDegs =
          ProjectileCalculator.estimateLeadYawDegrees(distanceMeters, tangentialVelocity);
      fieldTargetAngle = drive.getRotationToAllianceHub().plus(Rotation2d.fromDegrees(leadYawDegs));
    } else {
      Translation2d relativeSpeeds = drive.getTowerRelativeChassisSpeeds();
      tangentialVelocity = relativeSpeeds.getY();

      idealRps = ShooterConstants.PassRps;
      hoodDegs = HoodConstants.PassHoodDegs;
      double leadYawDegs = LEAD_YAW_COMPENSATION_INDEX * tangentialVelocity;
      fieldTargetAngle =
          drive.getRotationToAllianceTower().plus(Rotation2d.fromDegrees(leadYawDegs));
    }

    double turretRelativeDegs =
        fieldTargetAngle.minus(drive.getPose().getRotation()).getDegrees();
    double recommendedTurretDegs =
        turret.findNearestEquivalentAngle(
            turretRelativeDegs,
            turret.getCurrentPositionDegs(),
            TurretConstants.MinDegs,
            TurretConstants.MaxDegs);

    SmartDashboard.putNumber(KEY_TURRET, (int) recommendedTurretDegs);
    SmartDashboard.putNumber(KEY_HOOD, hoodDegs);
    SmartDashboard.putNumber(KEY_RPS, selectClosestPreset(idealRps));
  }

  private static double selectClosestPreset(double idealRps) {
    double best = MANUAL_RPS_PRESETS[0];
    double bestDiff = Math.abs(idealRps - best);
    for (int i = 1; i < MANUAL_RPS_PRESETS.length; i++) {
      double preset = MANUAL_RPS_PRESETS[i];
      double diff = Math.abs(idealRps - preset);
      if (diff < bestDiff || (Math.abs(diff - bestDiff) < 1e-9 && preset > best)) {
        best = preset;
        bestDiff = diff;
      }
    }
    return best;
  }

  private static void publishNaN() {
    SmartDashboard.putNumber(KEY_TURRET, Double.NaN);
    SmartDashboard.putNumber(KEY_HOOD, Double.NaN);
    SmartDashboard.putNumber(KEY_RPS, Double.NaN);
  }
}
