package frc.robot.util.simulation;

import frc.robot.Constants.ShooterConstants;

/**
 * <h3>Flywheel Speed → Ball Speed Physics Converter</h3>
 * 
 * <p><b>Physical Factors Considered:</b></p>
 * <ul>
 *   <li>Contact between flywheel surface and ball: rolling/slipping contact dynamics</li>
 *   <li>Friction transmission: momentum transfer via frictional force</li>
 *   <li>Slip loss: velocity reduction from ball slipping on flywheel surface</li>
 *   <li>Compression factor: energy loss due to contact deformation</li>
 * </ul>
 * 
 * <p><b>Model Basis:</b></p>
 * <pre>
 * contactSpeedMps = shooterRps × 2π × wheelRadius
 * ballSpeedMps = contactSpeedMps × transmissionCoeff
 * transmissionCoeff = frictionCoeff × (1 - slipLoss) × compressionFactor
 * </pre>
 */
public class ShooterPhysicsConverter {

    /**
     * <p>Converts flywheel rotational speed (RPS) to ball velocity (m/s).</p>
     * 
     * <p><b>Calculation Steps:</b></p>
     * <ol>
     *   <li>Calculate contact speed at flywheel surface: v_contact = shooterRps × 2π × wheelRadius</li>
     *   <li>Apply physical loss factors (friction, slip, compression)</li>
     *   <li>Final ball speed = contact speed × transmission coefficient</li>
     * </ol>
     * 
     * @param shooterRps flywheel speed in rotations per second
     * @return ball speed in meters per second
     */
    public static double rpsToMps(double shooterRps) {
        // Step 1: Calculate the contact speed at the flywheel surface
        // v_contact = ω × r = (shooterRps × 2π) × wheelRadius
        double wheelRadiusMeters = ShooterConstants.ShooterWheelRadiusMeters;
        double contactSpeedMps = shooterRps * 2.0 * Math.PI * wheelRadiusMeters;

        // Step 2: Apply physical loss factors
        //   - Friction coefficient: strength of friction between ball and flywheel
        //   - Slip loss: velocity reduction from ball slipping on flywheel surface
        //   - Compression factor: energy loss due to contact deformation
        double transmissionCoeff = ShooterConstants.FRICTION_COEFFICIENT
                * (1.0 - ShooterConstants.SLIP_LOSS_FACTOR)
                * ShooterConstants.COMPRESSION_FACTOR;

        // Step 3: Calculate final ball speed after applying transmission coefficient
        double ballSpeedMps = contactSpeedMps * transmissionCoeff;

        return ballSpeedMps;
    }

    /**
     * <p><b>Reverse Conversion:</b> Ball speed → Flywheel speed</p>
     * 
     * <p>Useful for debugging or calculating required motor speed from a target ball velocity.</p>
     * 
     * @param ballSpeedMps desired ball speed in meters per second
     * @return required flywheel speed in rotations per second
     */
    public static double mpsToRps(double ballSpeedMps) {
        double wheelRadiusMeters = ShooterConstants.ShooterWheelRadiusMeters;
        double transmissionCoeff = ShooterConstants.FRICTION_COEFFICIENT
                * (1.0 - ShooterConstants.SLIP_LOSS_FACTOR)
                * ShooterConstants.COMPRESSION_FACTOR;

        if (transmissionCoeff < 1e-6) {
            return 0.0;
        }

        double requiredContactSpeed = ballSpeedMps / transmissionCoeff;
        double requiredRps = requiredContactSpeed / (2.0 * Math.PI * wheelRadiusMeters);

        return requiredRps;
    }

    /**
     * <p>Calculates the energy transmission efficiency from flywheel to ball.</p>
     * 
     * <p>This is the product of all loss factors: friction × slip loss × compression.</p>
     * 
     * @return transmission coefficient, ranging from 0.0 (no transmission) to 1.0 (perfect transmission)
     */
    public static double getTransmissionCoefficient() {
        return ShooterConstants.FRICTION_COEFFICIENT
                * (1.0 - ShooterConstants.SLIP_LOSS_FACTOR)
                * ShooterConstants.COMPRESSION_FACTOR;
    }

    /**
     * <p>Calculates the linear velocity at the flywheel surface.</p>
     * 
     * <p>Useful for debugging and understanding the contact speed before applying loss factors.</p>
     * 
     * @param shooterRps flywheel rotational speed in rotations per second
     * @return linear velocity at the wheel surface in meters per second
     */
    public static double getWheelSurfaceSpeed(double shooterRps) {
        return shooterRps * 2.0 * Math.PI * ShooterConstants.ShooterWheelRadiusMeters;
    }
}
