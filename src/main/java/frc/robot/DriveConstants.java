package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.hardware.vendors.firstparties.ABC;

public class DriveConstants {
    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================
    public static final double kGearboxRatio = 8.46; // rotated 8.46 times, rotates gear that spins the wheel once
    public static final double kWheelDiameterInches = 6;
    public static final double kWheelDiameterFeet = 0.5;
    public static final double kWheelCircumferenceInches = kWheelDiameterInches * Math.PI;
    public static final double kWheelCircumferenceFeet = kWheelDiameterFeet * Math.PI;
    public static final double kReductionRatio = 0.01;
    public static final double kConversionFactor = kWheelCircumferenceInches * kReductionRatio;
    public static final double kMaxVelocityRPM = 5676;
    public static final double kPulsesPerRotation = 256.0;
    public static final double kCurrentLimitAmps = 60.0;
    public static final double kTrackWidthFeet = 20.875 / 12;
    public static final double kRpmToMpsFactor = (Units.feetToMeters(kWheelCircumferenceFeet) / kGearboxRatio) * 1 / 60;
    public static final double kMaxMpsVelocity = kRpmToMpsFactor * kMaxVelocityRPM;
    public static final int kEncoderCPR = 1024;
    public static final double kEncoderDistancePerPulse = (Units.feetToMeters(kTrackWidthFeet) * Math.PI) / (double) kEncoderCPR;
    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================


    //ALL PLACE HOLDER VALUES FROM WIPLIB
    //TODO: PUT IN REAL VALUES BY TESTING USING SYSIS
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
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
