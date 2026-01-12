package frc.robot.Library.team6940;

public class TrajectoryPoint {
  public final double time; // seconds
  public final double height; // meters
  public final double angleDeg; // degrees
  public final double dh; // elevator velocity
  public final double dtheta; // arm velocity (deg/s)

  public TrajectoryPoint(double time, double height, double angleDeg, double dh, double dtheta) {
    this.time = time;
    this.height = height;
    this.angleDeg = angleDeg;
    this.dh = dh;
    this.dtheta = dtheta;
  }
}
