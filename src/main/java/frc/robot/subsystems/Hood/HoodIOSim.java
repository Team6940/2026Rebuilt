package frc.robot.subsystems.Hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.HoodConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

public class HoodIOSim implements HoodIO {
  private final MapleMotorSim motorSim;
  private final GenericMotorController controller;
  private final PIDController positionPid = new PIDController(0.14, 0.0, 0.001);

  private double targetPositionDegs = HoodConstants.IdlePosition;
  private boolean closedLoop = false;
  private double positionOffsetDegs = 0.0;

  public HoodIOSim() {
    SimMotorConfigs configs =
        new SimMotorConfigs(
                DCMotor.getFalcon500(1),
                HoodConstants.HoodRatio,
                KilogramSquareMeters.of(0.02),
                Volts.of(12.0))
            .withHardLimits(Degrees.of(HoodConstants.MinDegs), Degrees.of(HoodConstants.MaxDegs));
    motorSim = new MapleMotorSim(configs);
    controller = motorSim.useSimpleDCMotorController();
    controller.withSoftwareLimits(
        Degrees.of(HoodConstants.MinDegs), Degrees.of(HoodConstants.MaxDegs));
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    controller.requestVoltage(Volts.of(voltage));
  }

  @Override
  public void setPosition(double positionDegrees) {
    closedLoop = true;
    targetPositionDegs = positionDegrees;
  }

  @Override
  public void resetPosition(double positionDegrees) {
    controller.requestVoltage(Volts.zero());
    double rawDegs = motorSim.getAngularPosition().in(Degrees);
    positionOffsetDegs = positionDegrees - rawDegs;
    targetPositionDegs = positionDegrees;
    positionPid.reset();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    Angle position = motorSim.getAngularPosition();
    double positionDegs = position.in(Degrees) + positionOffsetDegs;

    if (closedLoop) {
      double volts = positionPid.calculate(positionDegs, targetPositionDegs);
      volts = MathUtil.clamp(volts, -12.0, 12.0);
      controller.requestVoltage(Volts.of(volts));
    }

    Time dt = SimulatedArena.getSimulationDt();
    motorSim.update(dt);

    inputs.motorConnected = true;
    inputs.motorVoltageVolts = motorSim.getAppliedVoltage().in(Volts);
    inputs.motorCurrentAmps = motorSim.getStatorCurrent().in(Amps);
    inputs.hoodPositionDegrees = motorSim.getAngularPosition().in(Degrees) + positionOffsetDegs;
  }
}
