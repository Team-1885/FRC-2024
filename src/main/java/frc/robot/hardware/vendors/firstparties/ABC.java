package frc.robot.hardware.vendors.firstparties;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import edu.wpi.first.math.geometry.Rotation2d;
import static java.lang.Math.*;

public class ABC {
  // These are the variables used to access methods
  private Angle mAngle;
  private Distance mDistance;

  public Angle getAngle() {
    return mAngle;
  }

  public Distance getDistance() {
    return mDistance;
  }

  public static double rpm_to_rads_per_sec(double rpm) {
    return rpm * 2.0 * PI / 60.0;
  }

  public static double rads_per_sec_to_rpm(double rads_per_sec) {
    return rads_per_sec * 60.0 / (2.0 * Math.PI);
  }

  public static double inches_to_meters(double inches) {
    return inches * 0.0254;
  }

  public static double meters_to_inches(double meters) {
    return meters / 0.0254;
  }

  public static double feet_to_meters(double feet) {
    return inches_to_meters(feet * 12.0);
  }

  public static double meters_to_feet(double meters) {
    return meters_to_inches(meters) / 12.0;
  }

  public static double degrees_to_radians(double degrees) {
    return Rotation2d.fromDegrees(degrees).getRadians();
  }

  public static double radians_to_degrees(double radians) {
    return new Rotation2d(radians).getDegrees();
  }

  public static double degrees_to_rotations(double degrees) {
    return degrees / 360.0;
  }

  public static double rotations_to_degrees(double rotations) {
    return rotations * 360.0;
  }

  public static double degrees_to_rotations(double degrees, double smallToBigGearRatio) {
    return degrees_to_rotations(degrees) / smallToBigGearRatio;
  }

  public static double rotations_to_degrees(double rotations, double smallToBigGearRatio) {
    return rotations_to_degrees(rotations) * smallToBigGearRatio;
  }

  public class Angle {
    private final double mRadians;

    private Angle(double pRadians) {
      mRadians = pRadians;
    }

    public Angle add(Angle pOther) {
      return fromRadians(mRadians + pOther.mRadians);
    }

    public Angle subtract(Angle pOther) {
      return fromRadians(mRadians - pOther.mRadians);
    }

    public double radians() {
      return mRadians;
    }

    public double degrees() {
      return radians_to_degrees(mRadians);
    }

    public Angle fromNormal() {
      return fromRadians(PI / 2 - mRadians);
    }

    public Angle fromDegrees(double pDegrees) {
      return fromRadians(radians_to_degrees(pDegrees));
    }

    public Angle fromRadians(double pRadians) {
      return new Angle(pRadians);
    }
  }

  public class Distance {
    // meters
    private final double si;

    public NumberFormat nf = new DecimalFormat("0.00");

    public double FEET_PER_METER = 0.3048;
    public double INCHES_PER_FEET = 12;

    public double si() {
      return si;
    }

    public double meters() {
      return si;
    }

    public double feet() {
      return si * FEET_PER_METER;
    }

    public double inches() {
      return feet() * 12;
    }

    private Distance(double si) {
      this.si = si;
    }

    public Distance fromSI(double si) {
      return new Distance(si);
    }

    public Distance fromMeters(double meters) {
      return fromSI(meters);
    }

    public Distance fromFeet(double feet) {
      return fromMeters(feet / FEET_PER_METER);
    }

    public Distance fromInches(double inches) {
      return fromFeet(inches / INCHES_PER_FEET);
    }

    public String toString() {
      return nf.format(si);
    }

    // Some trig functions. Just add these as we need them.
    public Distance hypot(Distance x, Distance y) {
      return fromSI(Math.hypot(x.si, y.si));
    }

    public Angle atan2(Distance x, Distance y) {
      return new ABC.Angle(Math.atan2(y.si, x.si));
    }
  }

  public class Conversions {
    
  }
}
