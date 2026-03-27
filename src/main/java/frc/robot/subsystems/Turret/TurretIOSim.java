package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.TurretConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

public class TurretIOSim implements TurretIO {
  private enum ControlMode {
    VOLTAGE,
    POSITION,
    VELOCITY
  }

  private final MapleMotorSim motorSim;
  private final GenericMotorController controller;
  private final PIDController positionPid = new PIDController(0.12, 0.0, 0.001);
  private final PIDController velocityPid = new PIDController(0.02, 0.0, 0.0);

  private ControlMode mode = ControlMode.VOLTAGE;
  private double targetPositionDegs = 0.0;
  private double targetVelocityDegsPerSec = 0.0;
  private double positionOffsetDegs = 0.0;

  public TurretIOSim() {
    SimMotorConfigs configs =
        new SimMotorConfigs(
                DCMotor.getFalcon500(1),
                TurretConstants.TurretRatio,
                KilogramSquareMeters.of(0.08),
                Volts.of(12.0))
            .withHardLimits(
                Degrees.of(TurretConstants.MinDegs), Degrees.of(TurretConstants.MaxDegs));
    motorSim = new MapleMotorSim(configs);
    controller = motorSim.useSimpleDCMotorController();
    controller.withSoftwareLimits(
        Degrees.of(TurretConstants.MinDegs), Degrees.of(TurretConstants.MaxDegs));
  }

  @Override
  public void setVoltage(double voltage) {
    mode = ControlMode.VOLTAGE;
    controller.requestVoltage(Volts.of(voltage));
  }

  @Override
  public void setPosition(double positionDegrees) {
    mode = ControlMode.POSITION;
    targetPositionDegs = positionDegrees;
  }

  @Override
  public void setVelocity(double degsPerSec) {
    mode = ControlMode.VELOCITY;
    targetVelocityDegsPerSec = degsPerSec;
  }

  @Override
  public void resetPosition(double positionDegrees) {
    controller.requestVoltage(Volts.zero());
    // No direct setter for state; we align targets so the controller holds this position.
    double rawDegs = motorSim.getAngularPosition().in(Degrees);
    positionOffsetDegs = positionDegrees - rawDegs;
    targetPositionDegs = positionDegrees;
    positionPid.reset();
    velocityPid.reset();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    Angle position = motorSim.getAngularPosition();
    AngularVelocity velocity = motorSim.getVelocity();
    double positionDegs = position.in(Degrees) + positionOffsetDegs;
    double velocityDegsPerSec = velocity.in(DegreesPerSecond);

    Voltage commandedVoltage = Volts.zero();
    switch (mode) {
      case POSITION -> {
        double volts = positionPid.calculate(positionDegs, targetPositionDegs);
        commandedVoltage = Volts.of(MathUtil.clamp(volts, -12.0, 12.0));
      }
      case VELOCITY -> {
        double volts = velocityPid.calculate(velocityDegsPerSec, targetVelocityDegsPerSec);
        commandedVoltage = Volts.of(MathUtil.clamp(volts, -12.0, 12.0));
      }
      case VOLTAGE -> commandedVoltage = controller.getAppliedVoltage();
    }
    controller.requestVoltage(commandedVoltage);

    Time dt = SimulatedArena.getSimulationDt();
    motorSim.update(dt);

    inputs.motorConnected = true;
    inputs.encoderConnected = true;
    inputs.encoder2Connected = true;

    inputs.motorVoltageVolts = motorSim.getAppliedVoltage().in(Volts);
    inputs.motorStatorCurrentAmps = motorSim.getStatorCurrent().in(Amps);
    inputs.motorVelocityDegsPerSec = motorSim.getVelocity().in(DegreesPerSecond);

    inputs.turretPositionDegrees = motorSim.getAngularPosition().in(Degrees) + positionOffsetDegs;
    inputs.encoderPositionDegrees = inputs.turretPositionDegrees;
    inputs.encoder2PositionDegrees = inputs.turretPositionDegrees;

    inputs.encoderMagnetHealth = TurretIOInputs.EncoderMagnetHealth.GOOD;
    inputs.encoder2MagnetHealth = TurretIOInputs.EncoderMagnetHealth.GOOD;
  }
}
