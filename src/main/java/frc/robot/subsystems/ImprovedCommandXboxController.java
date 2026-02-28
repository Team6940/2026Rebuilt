package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

public class ImprovedCommandXboxController extends CommandXboxController {
    /** Represents a digital button on an XboxController. */
    public enum Button {
        kAutoButton(0),
        /** Left bumper. */
        kLeftBumper(5),
        /** Right bumper. */
        kRightBumper(6),
        /** Left stick. */
        kLeftStick(9),
        /** Right stick. */
        kRightStick(10),
        /** A. */
        kA(1),
        /** B. */
        kB(2),
        /** X. */
        kX(3),
        /** Y. */
        kY(4),
        /** Back. */
        kBack(7),
        /** Start. */
        kStart(8),
        /** Left trigger. */
        kLeftTrigger(15),
        /** Right trigger. */
        kRightTrigger(16);

        /** Button value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods. This is done by
         * stripping the leading `k`, and if not a Bumper button append `Button`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("Bumper")) {
                return name;
            }
            if (name.endsWith("Trigger")) {
                return name;
            }
            return name + "Button";
        }
    }
    private double m_triggerThreshold = 0.0;
    public ImprovedCommandXboxController(final int port) {
        super(port);
        m_triggerThreshold = 0.5;
    }
    public ImprovedCommandXboxController(final int port, final double _triggerThreshold) {
        super(port);
        m_triggerThreshold = _triggerThreshold;
    }
    /**
     * Read the value of the left trigger (LT) button on the controller.
     *
     * @return The axis of the trigger is greater than the trigger threshold
     */
    public boolean getLeftTrigger(){
        return getLeftTriggerAxis() > m_triggerThreshold;
    }
    /**
     * Read the value of the right trigger (RT) button on the controller.
     *
     * @return The axis of the trigger is greater than the trigger threshold
     */
    public boolean getRightTrigger(){
        return getRightTriggerAxis() > m_triggerThreshold;
    }
    /**
     * Get the button value (starting at button 1).
     *
     * <p>The buttons are returned in a single 16 bit value with one bit representing the state of
     * each button. The appropriate button is returned as a boolean value.
     *
     * <p>This method returns true if the button is being held down at the time that this method is
     * being called.
     *
     * @param button The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getButton(int button){
        if(button==Button.kAutoButton.value) {
            return true;
        }
        if(button == Button.kLeftTrigger.value){
            return getLeftTrigger();
        }
        if(button == Button.kRightTrigger.value){
            return getRightTrigger();
        }
        return getHID().getRawButton(button);
    }

    public boolean getButton(Button button){
        return getButton(button.value);
    }

    /**
     * THIS DOES NOT APPLY TO AUTOBUTTON OR TRIGGERS!
     * @param button
     * @return
     */
    public boolean getButtonPressed(int button){
        return getHID().getRawButtonPressed(button);
    }

    /**
     * THIS DOES NOT APPLY TO TRIGGERS!
     * @param button
     * @return
     */
    public boolean getButtonPressed(Button button){
        return getButtonPressed(button.value);
    }

    /**
     * Returns the angle of the joystick in degrees, where 0 degrees is straight up, and increases clockwise.
     * @param x The x-axis value of the joystick.
     * @param y The y-axis value of the joystick.
     * @param deadzone The deadzone radius to prevent noise when the stick is near the center.
     * @return The angle of the joystick in degrees, normalized to [-180, 180].
     */
    public double getJoystickAngleDeg180(double x, double y, double deadzone) {
        if (Math.hypot(x, y) < deadzone) return 0.0; // Deadzone check to prevent noise when the stick is near the center
        double angleRad = Math.atan2(x, y);
        double angleDeg = Math.toDegrees(angleRad);
        return MathUtil.angleModulus(Math.toRadians(angleDeg)) * 180.0 / Math.PI;
    }

    /**
    * Returns the angle of the joystick in degrees, where 0 degrees is straight up, and increases clockwise.
    * @param x The x-axis value of the joystick.
    * @param y The y-axis value of the joystick.
    * @param deadzone The deadzone radius to prevent noise when the stick is near the center.
    * @return The angle of the joystick in degrees, normalized to [0, 360).
    */
    public double getJoystickAngleDeg360(double x, double y, double deadzone) {
        if (Math.hypot(x, y) < deadzone) return 0.0;
        double angleRad = Math.atan2(x, y);
        double angleDeg = Math.toDegrees(angleRad);
        return (angleDeg + 360.0) % 360.0; // Normalize to [0, 360)
    }

    // public ChassisSpeeds getDesiredRelativeSpeeds(){
    //     return new ChassisSpeeds(this.getLeftY(), this.getLeftX(), -this.getRightX() * 10.);
    // }

    /**
     * Applies a deadband and a power curve to a raw joystick axis value, preserving sign.
     * <p>Formula: {@code sign(raw) * pow(|applyDeadband(raw)|, power)}
     * <p>Uses {@link OperatorConstants#DEADBAND} and {@link OperatorConstants#INPUT_POWER}.
     *
     * @param raw The raw axis value in the range [-1, 1].
     * @return The shaped value in the range [-1, 1].
     */
    public static double applyInputCurve(double raw) {
        double db = MathUtil.applyDeadband(raw, OperatorConstants.DEADBAND);
        return Math.copySign(Math.pow(Math.abs(db), OperatorConstants.INPUT_POWER), db);
    }
}
