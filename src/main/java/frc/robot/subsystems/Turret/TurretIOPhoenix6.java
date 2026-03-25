package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.TurretConstants;

public class TurretIOPhoenix6 implements TurretIO {
  private static final TalonFX motor = new TalonFX(MotorIDs.TurretMotorID, new CANBus("canivore"));
  private static final CANcoder encoder =
      new CANcoder(MotorIDs.TurretEncoderID, new CANBus("canivore"));
  private static final CANcoder encoder2 =
      new CANcoder(MotorIDs.TurretEncoder2ID, new CANBus("canivore"));
  private static final MotionMagicTorqueCurrentFOC positionRequest =
      new MotionMagicTorqueCurrentFOC(0);
  private static final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0.0);

  public TurretIOPhoenix6() {
    encoderConfig();
    motorConfig();
  }

  private void encoderConfig() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset =
        Units.degreesToRotations(TurretConstants.TurretEncoderOffsetDegrees);
    config.MagnetSensor.SensorDirection = TurretConstants.TurretEncoderDirection;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    encoder.getConfigurator().apply(config);

    CANcoderConfiguration config2 = new CANcoderConfiguration();
    config2.MagnetSensor.MagnetOffset =
        Units.degreesToRotations(TurretConstants.TurretEncoder2OffsetDegrees);
    config2.MagnetSensor.SensorDirection = TurretConstants.TurretEncoder2Direction;
    config2.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    encoder2.getConfigurator().apply(config2);
  }

  private void motorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = TurretConstants.TurretRatio;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRemoteSensorID = MotorIDs.TurretEncoderID;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kI = TurretConstants.kI;
    config.Slot0.kD = TurretConstants.kD;
    config.Slot0.kV = TurretConstants.kV;
    config.Slot0.kS = TurretConstants.kS;

    config.Slot1.kP = TurretConstants.kP_velocity;
    config.Slot1.kV = TurretConstants.kV_velocity;
    config.Slot1.kS = TurretConstants.kS_velocity;

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
    motor.setControl(positionRequest.withPosition(Units.degreesToRotations(positionDegrees)));
  }

  @Override
  public void setVelocity(double degsPerSec) {
    motor.setControl(
        velocityRequest.withVelocity(Units.degreesToRotations(degsPerSec)).withSlot(1));
  }

  @Override
  public void resetPosition(double positionDegrees) {
    motor.setPosition(Units.degreesToRotations(positionDegrees));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(),
                motor.getSupplyCurrent(),
                motor.getPosition(),
                motor.getVelocity())
            .isOK();

    inputs.encoderConnected =
        BaseStatusSignal.refreshAll(
                encoder.getAbsolutePosition(), encoder.getPosition(), encoder.getMagnetHealth())
            .isOK();

    inputs.encoder2Connected =
        BaseStatusSignal.refreshAll(
                encoder2.getAbsolutePosition(), encoder2.getPosition(), encoder2.getMagnetHealth())
            .isOK();

    inputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorStatorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.motorVelocityDegsPerSec =
        Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
    inputs.turretPositionDegrees = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    inputs.encoderPositionDegrees =
        Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    inputs.encoder2PositionDegrees =
        Units.rotationsToDegrees(encoder2.getAbsolutePosition().getValueAsDouble());
    // The calculate position algorithm requires the absolute position of the encoder

    switch (encoder.getMagnetHealth().getValue()) {
      case Magnet_Green -> inputs.encoderMagnetHealth = TurretIOInputs.EncoderMagnetHealth.GOOD;
      case Magnet_Orange -> inputs.encoderMagnetHealth = TurretIOInputs.EncoderMagnetHealth.RISKY;
      case Magnet_Red -> inputs.encoderMagnetHealth = TurretIOInputs.EncoderMagnetHealth.BAD;
      case Magnet_Invalid ->
          inputs.encoderMagnetHealth = TurretIOInputs.EncoderMagnetHealth.INVALID;
      default -> {}
    }

    switch (encoder2.getMagnetHealth().getValue()) {
      case Magnet_Green -> inputs.encoder2MagnetHealth = TurretIOInputs.EncoderMagnetHealth.GOOD;
      case Magnet_Orange -> inputs.encoder2MagnetHealth = TurretIOInputs.EncoderMagnetHealth.RISKY;
      case Magnet_Red -> inputs.encoder2MagnetHealth = TurretIOInputs.EncoderMagnetHealth.BAD;
      case Magnet_Invalid ->
          inputs.encoder2MagnetHealth = TurretIOInputs.EncoderMagnetHealth.INVALID;
      default -> {}
    }
  }
}
