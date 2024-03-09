// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

@SuppressWarnings({ "PMD.CommentSize", "PMD.LongVariable" })
public final class RobotMap {

  public static final class DriveConstants {
    public static final double kTrackWidthMeters = 0.530225;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically for *your* robot's drive. The Robot Characterization Toolsuite provides a convenient tool for obtaining these values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    public static final double kGearRatio = 8.46;
    public static final double kWheelRadiusInches = 3;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

    public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 2;
    public static final int kOperatorControllerPort = 3;
  }

  public static class DrivetrainConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 3;
    public static final int kLeftFrontID = 4;
    public static final int kRightRearID = 1;
    public static final int kRightFrontID = 2;

    // Current limit for drivetrain motors
    public static final int kCurrentLimit = 60;

    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================
     
    public static final double kWheelDiameterFeet = 0.5;
    public static final double kWheelCircumferenceFeet = Units.feetToMeters(kWheelDiameterFeet * Math.PI);
    public static final double kMaxVelocityRPM = 5676;
    public static final double kPulsesPerRotation = 256.0;
    public static final double kTrackWidthMeters = Units.feetToMeters(20.875 / 12);
    public static final int kEncoderCPR = 1024;
    public static final double kEncoderDistancePerPulse = (kTrackWidthMeters * Math.PI) / (double) kEncoderCPR;

    //EVERY CONSTANT AUTON USES
    public static final double kWheelDiameterInches = 6;
    public static final double kGearboxRatio = 1/8.46; // rotated 8.46 times, rotates gear that spins the wheel once
    public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kLinearDistanceConversionFactor =  kWheelCircumferenceMeters * kGearboxRatio;
    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================


    //ALL PLACE HOLDER VALUES FROM WIPLIB
    // TODO: PUT IN REAL VALUES BY TESTING USING SYSIS
    public static final double ksVolts = 0.25;
    public static final double kvVoltSecondsPerMeter = 2.16;
    public static final double kaVoltSecondsSquaredPerMeter = 0.48;
    public static final double kPDriveVel = 8.5;


    // DifferentialDriveKinematics
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);

    // Max Trajectory Velocity/Acceleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Ramsete Parameters
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    

  }

  public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kFeederID = 5;
    public static final int kLauncherID = 6;

    // Current limit for launcher and feed wheels
    public static final int kLauncherCurrentLimit = 80;
    public static final int kFeedCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kLauncherSpeed = 1;
    public static final double kLaunchFeederSpeed = 1;
    public static final double kIntakeLauncherSpeed = -1;
    public static final double kIntakeFeederSpeed = -.2;

    public static final double kLauncherDelay = 1;
  }

  public static class IntakeConstants
  {
    public static final double kTrueIntakeFeederSpeed = 1;
    public static final double kIntakeRotaterSpeed = 1;
  }
}
