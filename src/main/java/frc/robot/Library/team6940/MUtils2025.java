package frc.robot.Library.team6940;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

// This is a legacy version of MUtils, which is used in 2025. Author is lpx.
public class MUtils2025 {
  /**
   * Limits the given number within a given range.
   *
   * @param _Min
   * @param _Max
   * @param _NumberToLimit
   * @return the limited number
   */
  public static double numberLimit(double _Min, double _Max, double _NumberToLimit) {
    double _LimitedNumber = _NumberToLimit;
    _LimitedNumber = Math.min(_Max, _LimitedNumber);
    _LimitedNumber = Math.max(_Min, _LimitedNumber);
    return _LimitedNumber;
  }

  /**
   * Limits the given number within a given range.
   *
   * @param _Min
   * @param _Max
   * @param _NumberToLimit
   * @return the limited number
   */
  public static int numberLimit(int _Min, int _Max, int _NumberToLimit) {
    int _LimitedNumber = _NumberToLimit;
    _LimitedNumber = Math.min(_Max, _LimitedNumber);
    _LimitedNumber = Math.max(_Min, _LimitedNumber);
    return _LimitedNumber;
  }

  /**
   * Cycles a number within a given range.
   *
   * <p>This method assumes that the range is inclusive.
   *
   * <p>I didn't do a lot of safety checks here, do not use this method with untrusted inputs.
   *
   * <p>debug it, I don't know if it works in the negative field
   * @param currentNumber
   * @param start the smallest number in the range
   * @param end the biggest number in the range
   * @param delta the amount to change the number by
   * @return returns the changed number
   */
  public static int cycleNumber(int currentNumber, int start, int end, int delta) {
    int length = end - start + 1;
    currentNumber -= start; // thus we have a number between 0 and length-1
    delta %= length;
    currentNumber %= length; // safety check
    return (currentNumber + length + delta) % length + start;
  }

  public static double dotProduct(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  /**
   * In a y=kx+b defined on [m, n] style
   *
   * <p>Be aware that if your segment is parallel to the y-axis, find yourself a way around this.
   */
  public static class SegmentOnTheField {
    public double k, b, m, n;

    private void validateMandN() {
      if (m > n) {
        double temp = m;
        m = n;
        n = temp;
      }
    }

    /**
     * Constructor
     *
     * @param k the slope of the line
     * @param b the y-intercept of the line
     * @param m the x-coordinate of the start point of the segment
     * @param n the x-coordinate of the end point of the segment should be greater than m
     */
    public SegmentOnTheField(double k, double b, double m, double n) {
      this.k = k;
      this.b = b;
      this.m = m;
      this.n = n;
      validateMandN();
    }

    public SegmentOnTheField(Translation2d p1, Translation2d p2) {
      double k = (p2.getY() - p1.getY()) / (p2.getX() - p1.getX());
      double b = p1.getY() - k * p1.getX();
      this.k = k;
      this.b = b;
      this.m = p1.getX();
      this.n = p2.getX();
      validateMandN();
    }

    /**
     * Calculates the point on the line of the segment given an x-coordinate.
     *
     * @param x
     * @return the point on the line of the segment given an x-coordinate.
     */
    public Translation2d calculate(double x) {
      return new Translation2d(x, k * x + b);
    }

    public Translation2d solve(double y) {
      return new Translation2d((y - b) / k, y);
    }

    public Translation2d getStartPoint() {
      return calculate(m);
    }

    public Translation2d getEndPoint() {
      return calculate(n);
    }

    /**
     * Calculates the projection of a point on the line of the segment.
     *
     * @param point
     * @return the projection of a point on the line of the segment.
     */
    public Translation2d getProjection(Translation2d point) {
      return calculate((point.getX() + k * point.getY() - k * b) / (k * k + 1));
    }

    public SegmentOnTheField getVerticalSegment(Translation2d point) {
      return new SegmentOnTheField(point, getProjection(point));
    }

    public double getDistanceTo(Translation2d point) {
      return point.getDistance(getProjection(point));
    }

    public boolean isPointOnSegment(Translation2d point) {
      return getDistanceTo(point) <= 0.001 && point.getX() >= m && point.getX() <= n;
    }

    public boolean isPointOnLine(Translation2d point) {
      return getDistanceTo(point) <= 0.001;
    }

    public Translation2d getNearestPointTo(Translation2d point) {
      Translation2d projection = getProjection(point);
      if (isPointOnSegment(projection)) {
        return projection;
      } else {
        double distanceToStart = point.getDistance(calculate(m));
        double distanceToEnd = point.getDistance(calculate(n));
        return distanceToStart < distanceToEnd ? calculate(m) : calculate(n);
      }
    }

    public Translation2d getMidPoint() {
      return calculate((m + n) / 2);
    }

    public double getLength() {
      return getStartPoint().getDistance(getEndPoint());
    }
  }

  public static Pose2d transformFieldRelative(Pose2d p, Transform2d t) {
    return new Pose2d(
        p.getTranslation().plus(t.getTranslation()), p.getRotation().plus(t.getRotation()));
  }

  public static Pose2d transformRobotRelative(Pose2d p, Transform2d t) {
    return p.plus(t);
  }
}
