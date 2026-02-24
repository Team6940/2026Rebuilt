package frc.robot.subsystems.Drive;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveDriveSendable implements Sendable {
  private static final int FRONT_LEFT = 0;
  private static final int FRONT_RIGHT = 1;
  private static final int BACK_LEFT = 2;
  private static final int BACK_RIGHT = 3;

  private Drive drive = Drive.getInstance();
  
    public SwerveDriveSendable(Drive drive) {
      this.drive = drive;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty("Front Left Angle",
        () -> drive.getModuleAngleRad(FRONT_LEFT), null);
    builder.addDoubleProperty("Front Left Velocity",
        () -> drive.getModuleVelocityMps(FRONT_LEFT), null);

    builder.addDoubleProperty("Front Right Angle",
        () -> drive.getModuleAngleRad(FRONT_RIGHT), null);
    builder.addDoubleProperty("Front Right Velocity",
        () -> drive.getModuleVelocityMps(FRONT_RIGHT), null);

    builder.addDoubleProperty("Back Left Angle",
        () -> drive.getModuleAngleRad(BACK_LEFT), null);
    builder.addDoubleProperty("Back Left Velocity",
        () -> drive.getModuleVelocityMps(BACK_LEFT), null);

    builder.addDoubleProperty("Back Right Angle",
        () -> drive.getModuleAngleRad(BACK_RIGHT), null);
    builder.addDoubleProperty("Back Right Velocity",
        () -> drive.getModuleVelocityMps(BACK_RIGHT), null);

    builder.addDoubleProperty("Robot Angle",
        () -> drive.getRotation().getRadians(), null);
  }
}
