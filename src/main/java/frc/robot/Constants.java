// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 2;
    public static final int kOperatorControllerPort = 3;
  }

  public static class DrivetrainConstants {
    
    
    // CAN IDs for motor controllers
    public static final int kLeftMasterID = 6;
    public static final int kLeftFollowerID = 7;
    public static final int kRightMasterID = 8;
    public static final int kRightFollowerID = 9;

    

    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================
    public static final int kCurrentLimit = 40;
    public static final double kWheelDiameterFeet = 0.5;
    public static final double kWheelCircumferenceFeet = Units.feetToMeters(kWheelDiameterFeet * Math.PI);
    public static final double kMaxVelocityRPM = 5676;
    public static final double kTrackWidthMeters = Units.feetToMeters(20.875 / 12);
    public static final int kEncoderCPR = 1024;

    //EVERY CONSTANT AUTON USES
    public static final double kWheelDiameterInches = 6;
    public static final double kGearboxRatio = 1/8.46; // rotated 8.46 times, rotates gear that spins the wheel once
    public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kLinearDistanceConversionFactor =  kWheelCircumferenceMeters * kGearboxRatio;
    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================

    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0;
    
    public static final double kTurnP = 0.033;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    
    public static final double ksVolts = 0.25;
    public static final double kvVoltSecondsPerMeter = 2.16;
    public static final double kaVoltSecondsSquaredPerMeter = 0.48;
    public static final double kPDriveVel = 8.5; // Before was 8.5


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
    public static final int kFeederID = 3; // BEFORE 2
    public static final int kLauncherID = 2; // BEFORE 3

    // Current limit for launcher and feed wheels
    public static final int kLauncherCurrentLimit = 80;
    public static final int kFeedCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels in reverse
    public static final double kLauncherSpeed = 1d;
    public static final double kLaunchFeederSpeed = -1d;
    public static final double kIntakeLauncherSpeed = -1d;
    public static final double kIntakeFeederSpeed = -1d;
    public static final double kIntakeFeederSpeedSlower = -0.2d;
    public static final double kFeedLaunchSpeed = 0.5d;
    public static final double kStopSpeed = 0.0d;

    public static final double kLauncherDelay = 1d;
  }

  public static class IntakeConstants
  {
    public static final int kFeederID = 1; // BEFORE 1
    public static final int kRotatorMasterID = 1; // BEFORE 1
    public static final int kRotatorFollowerID = 2; // BEFORE 2
    public static final double kTrueIntakeFeederSpeed = 1;
    public static final double kIntakeRotaterSpeed = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class ClimberConstants {
    public static final int climber1ID = 24;
    public static final int climber2ID = 25;

    public static final double speedRight = 0.5;
    public static final double speedLeft = speedRight; //0.395;
    
    
  
  }
}