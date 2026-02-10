package frc.robot.util.turretAiming.mobileScoring;

import frc.robot.util.turretAiming.constants.TurretConstants;

class BallisticSolution {
    public final double launchVelocity;   // (m/s)
    public final double elevationAngle;   // (deg)
    public final double azimuthAngle;     // (deg)

    public BallisticSolution(double velocity, double elevation, double azimuth) {
        this.launchVelocity = velocity;
        this.elevationAngle = elevation;
        this.azimuthAngle = azimuth;
    }
}

/**
 * Turret aiming with XY-only chassis velocity compensation.
 * - Physics: gravity-only ballistic, no air drag
 * - Compensation: iteratively forward-predict the chassis XY displacement during flight time
 * - Output: launch velocity, elevation (deg), azimuth (deg)
 */
public class TurretInverseSolutionCalculator {
    private static final double G = 9.81;
    private static final double MAX_ELEVATION = TurretConstants.MAX_ELEVATION;
    private static final double MAX_VELOCITY = TurretConstants.MAX_VELOCITY;

    private double targetX, targetY, targetZ;
    private boolean isTargetTooFar = false;
    private boolean isTargetTooClose = false;

    public void setTarget(double x, double y, double z) {
        this.targetX = x;
        this.targetY = y;
        this.targetZ = z;
    }

    /**
     * Returns { horizontal distance d, height diff h, dx, dy }
     */
    private double[] calculateTargetParameters(double turretX, double turretY, double turretZ) {
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        double dz = targetZ - turretZ;
        double horizontalDistance = Math.sqrt(dx * dx + dy * dy);
        return new double[]{horizontalDistance, dz, dx, dy};
    }

    /**
     * Minimal velocity bound (gravity-only, no drag). Returns Double.MAX_VALUE if no geometric solution.
     */
    private double calculateMinimumVelocity(double d, double h) {
        double denominator = -h + Math.sqrt(h * h + d * d);
        if (denominator <= 0) return Double.MAX_VALUE;
        return d * Math.sqrt(G / denominator);
    }

    /**
     * Static (no chassis speed) high-trajectory solution for a given v.
     * Returns { thetaDeg, flightTime } or null if infeasible.
     */
    private double[] calculateTrajectory(double v, double d, double h) {
        double A = (G * d * d) / (2 * v * v);
        double discriminant = d * d - 4 * A * (h + A);
        if (discriminant < 0) return null;

        // High-trajectory branch
        double u = (d + Math.sqrt(discriminant)) / (2 * A);
        double thetaRad = Math.atan(u);
        double thetaDeg = Math.toDegrees(thetaRad);
        if (thetaDeg < 0 || thetaDeg > MAX_ELEVATION) return null;

        double cosTheta = Math.cos(thetaRad);
        double flightTime = d / (v * cosTheta);
        return new double[]{thetaDeg, flightTime};
    }

    /**
     * Binary search over velocity to minimize flight time for static geometry (no chassis speed inside).
     */
    private BallisticSolution binarySearchOptimalSolution(double d, double h, double dx, double dy,
                                                          double vMin, double vMax) {
        int left = (int) Math.ceil(vMin);
        int right = (int) Math.floor(vMax);
        double bestVelocity = -1, bestAngle = -1, minTime = Double.MAX_VALUE;

        while (left <= right) {
            int mid = (left + right) / 2;
            double[] result = calculateTrajectory(mid, d, h);
            if (result != null) {
                double flightTime = result[1];
                if (flightTime < minTime) {
                    minTime = flightTime;
                    bestVelocity = mid;
                    bestAngle = result[0];
                }
                right = mid - 1;
            } else {
                left = mid + 1;
            }
        }

        if (bestVelocity != -1) {
            double azimuthDeg = Math.toDegrees(Math.atan2(dy, dx));
            return new BallisticSolution(bestVelocity, bestAngle, azimuthDeg);
        }
        return null;
    }

    /**
     * XY-only chassis velocity compensation.
     * Iteratively forward-predict chassis displacement during flight, update geometry, and re-solve.
     *
     * turretVx, turretVy: chassis/turret XY velocity in world frame (m/s)
     * Note: turretVz is ignored (assumed 0) in this XY-only model.
     */
    public BallisticSolution calculateOptimalTrajectory(double turretX, double turretY, double turretZ,
                                                        double turretVx, double turretVy) {
        isTargetTooFar = false;
        isTargetTooClose = false;

        double[] params = calculateTargetParameters(turretX, turretY, turretZ);
        double d = params[0], h = params[1], dx = params[2], dy = params[3];

        // Target vertically above (or coincident) and non-negative height -> treat as too close
        if (d < 1e-6 && h >= 0) {
            isTargetTooClose = true;
            return new BallisticSolution(Double.NaN, Double.NaN, Double.NaN);
        }

        double vMin = calculateMinimumVelocity(d, h);
        if (vMin == Double.MAX_VALUE || vMin > MAX_VELOCITY) {
            isTargetTooFar = true;
            return new BallisticSolution(Double.NaN, Double.NaN, Double.NaN);
        }

        BallisticSolution bestSolution = null;
        double bestError = Double.MAX_VALUE;

        // Iterative XY forward-prediction compensation
        for (int iter = 0; iter < 5; iter++) {
            BallisticSolution sol = binarySearchOptimalSolution(d, h, dx, dy, vMin, MAX_VELOCITY);
            if (sol == null) {
                isTargetTooClose = true;
                return new BallisticSolution(Double.NaN, Double.NaN, Double.NaN);
            }

            double thetaRad = Math.toRadians(sol.elevationAngle);
            double cosTheta = Math.cos(thetaRad);
            double flightTime = d / (sol.launchVelocity * cosTheta);

            // Chassis displacement during flight (XY only)
            double turretDx = turretVx * flightTime;
            double turretDy = turretVy * flightTime;

            // Update relative geometry
            double newDx = targetX - (turretX + turretDx);
            double newDy = targetY - (turretY + turretDy);
            double newD = Math.sqrt(newDx * newDx + newDy * newDy);
            double newH = h; // no Z chassis motion

            double error = Math.abs(newD - d) + Math.abs(newH - h);
            if (error < bestError) {
                bestError = error;
                bestSolution = sol;
            }

            d = newD;
            h = newH;
            dx = newDx;
            dy = newDy;
        }

        return bestSolution;
    }

    public boolean isTargetTooFar() { return isTargetTooFar; }
    public boolean isTargetTooClose() { return isTargetTooClose; }
}
