package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.TurretConstants;

public class TurretIOPhoenix6 implements TurretIO {
  private static final TalonFX motor = new TalonFX(MotorIDs.TurretMotorID, CANBus.roboRIO());
  private static final CANcoder encoder = new CANcoder(MotorIDs.TurretEncoderID, CANBus.roboRIO());
  private static final MotionMagicVoltage request = new MotionMagicVoltage(0.0);

  public TurretIOPhoenix6() {
    encoderConfig();
    motorConfig();
  }

  private void encoderConfig() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset =
        Units.degreesToRotations(TurretConstants.TurretEncoderOffsetDegrees);
    config.MagnetSensor.SensorDirection = TurretConstants.TurretEncoderDirection;
    encoder.getConfigurator().apply(config);
  }

  private void motorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = TurretConstants.TurretEncoderToMechanismRatio;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = MotorIDs.TurretEncoderID;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kI = TurretConstants.kI;
    config.Slot0.kD = TurretConstants.kD;
    config.Slot0.kV = TurretConstants.kV;
    config.Slot0.kS = TurretConstants.kS;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = TurretConstants.TurretSupplyCurrentLimit;

    config.MotorOutput.Inverted = TurretConstants.Inverted;

    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MaxVelocity;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.Acceleration;

    motor.getConfigurator().apply(config);
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
    encoder.setPosition(Units.degreesToRotations(positionDegrees));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getPosition())
            .isOK();

    inputs.encoderConnected =
        BaseStatusSignal.refreshAll(
                encoder.getAbsolutePosition(), encoder.getPosition(), encoder.getMagnetHealth())
            .isOK();

    inputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.turretPositionDegrees = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    inputs.encoderPositionDegrees =
        Units.rotationsToDegrees(encoder.getPosition().getValueAsDouble());

    switch (encoder.getMagnetHealth().getValue()) {
      case Magnet_Green -> inputs.encoderMagnetHealth = TurretIOInputs.EncoderMagnetHealth.GOOD;
      case Magnet_Orange -> inputs.encoderMagnetHealth = TurretIOInputs.EncoderMagnetHealth.RISKY;
      case Magnet_Red -> inputs.encoderMagnetHealth = TurretIOInputs.EncoderMagnetHealth.BAD;
      case Magnet_Invalid ->
          inputs.encoderMagnetHealth = TurretIOInputs.EncoderMagnetHealth.INVALID;
      default -> {}
    }
  }
}
