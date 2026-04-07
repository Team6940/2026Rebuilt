package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.ShooterConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

public class ShooterIOSim implements ShooterIO {
  private final MapleMotorSim motorSim;
  private final GenericMotorController controller;
  private final PIDController velocityPid = new PIDController(0.2, 0.0, 0.0);

  private double targetRps = 0.0;
  private boolean closedLoop = false;

  public ShooterIOSim() {
    SimMotorConfigs configs =
        new SimMotorConfigs(
            DCMotor.getFalcon500(1),
            ShooterConstants.ShooterRatio,
            KilogramSquareMeters.of(0.01),
            Volts.of(12.0));
    motorSim = new MapleMotorSim(configs);
    controller = motorSim.useSimpleDCMotorController();
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    controller.requestVoltage(Volts.of(voltage));
  }

  @Override
  public void setRPS(double rps) {
    targetRps = rps;
    closedLoop = true;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    AngularVelocity velocity = motorSim.getVelocity();
    double currentRps = velocity.in(RadiansPerSecond) / (2.0 * Math.PI);

    if (closedLoop) {
      double volts = velocityPid.calculate(currentRps, targetRps);
      volts = MathUtil.clamp(volts, -12.0, 12.0);
      controller.requestVoltage(Volts.of(volts));
    }

    Time dt = SimulatedArena.getSimulationDt();
    motorSim.update(dt);

    inputs.motorConnected = true;
    inputs.motorVoltageVolts = motorSim.getAppliedVoltage().in(Volts);
    inputs.motorSupplyCurrentAmps = motorSim.getSupplyCurrent().in(Amps);
    inputs.motorStatorCurrentAmps = motorSim.getStatorCurrent().in(Amps);
    inputs.motorTorqueCurrentAmps = 0.0;
    inputs.shooterVelocityRPS =
        motorSim.getVelocity().in(RadiansPerSecond) / (2.0 * Math.PI);

    // Follower mirrors main motor in sim
    inputs.followerConnected = true;
    inputs.followerVoltageVolts = inputs.motorVoltageVolts;
    inputs.followerSupplyCurrentAmps = inputs.motorSupplyCurrentAmps;
    inputs.followerStatorCurrentAmps = inputs.motorStatorCurrentAmps;
    inputs.followerTorqueCurrentAmps = inputs.motorTorqueCurrentAmps;
    inputs.followerVelocityRPS = inputs.shooterVelocityRPS;
  }
}
