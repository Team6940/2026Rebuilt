package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorIDs;

public class HoodIOPhoenix6 implements HoodIO {
  private static TalonFX motor = new TalonFX(MotorIDs.HoodMotorID, CANBus.roboRIO());
  private static final MotionMagicVoltage request = new MotionMagicVoltage(0.0).withEnableFOC(true);

  public HoodIOPhoenix6() {
    motorConfig();
  }

  private void motorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = HoodConstants.HoodRatio;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.Slot0.kP = HoodConstants.kP;
    config.Slot0.kI = HoodConstants.kI;
    config.Slot0.kD = HoodConstants.kD;
    config.Slot0.kV = HoodConstants.kV;
    config.Slot0.kS = HoodConstants.kS;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HoodSupplyCurrentLimit;

    config.MotorOutput.Inverted = HoodConstants.Inverted;

    config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.MaxVelocity;
    config.MotionMagic.MotionMagicAcceleration = HoodConstants.Acceleration;

    motor.getConfigurator().apply(config);
    motor.setPosition(Units.degreesToRotations(HoodConstants.IdlePosition));
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setPosition(double positionDegrees) {
    motor.setControl(request.withPosition(Units.degreesToRotations(positionDegrees)));
  }

  @Override
  public void resetPosition(double positionDegrees) {
    motor.setPosition(Units.degreesToRotations(positionDegrees));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getPosition())
            .isOK();

    inputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.hoodPositionDegrees = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }
}
