package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import frc.robot.hardware.vendors.firstparties.ABC;

public class DriveConstants {
    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================
    public static final double kGearboxRatio = 8.46; // rotated 8.46 times, rotates gear that spins the wheel once
    public static final double kWheelDiameterInches = 6;
    public static final double kWheelDiameterFeet = 0.5;
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(kWheelDiameterInches * Math.PI);
    public static final double kWheelCircumferenceFeet = Units.feetToMeters(kWheelDiameterFeet * Math.PI);
    public static final double kReductionRatio = 0.01;
    public static final double kConversionFactor = kWheelCircumferenceMeters * kReductionRatio;
    public static final double kMaxVelocityRPM = 5676;
    public static final double kPulsesPerRotation = 256.0;
    public static final double kCurrentLimitAmps = 60.0;
    public static final double kTrackWidthMeters = Units.feetToMeters(20.875 / 12);
    public static final double kRpmToMpsFactor = (Units.feetToMeters(kWheelCircumferenceFeet) / kGearboxRatio) * 1 / 60;
    public static final double kMaxMpsVelocity = kRpmToMpsFactor * kMaxVelocityRPM;
    public static final int kEncoderCPR = 1024;
    public static final double kEncoderDistancePerPulse = (kTrackWidthMeters * Math.PI) / (double) kEncoderCPR;
    public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters(kWheelDiameterInches) * Math.PI) / kGearboxRatio;
    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================
    public static final double gearboxRatio = 1/8.46;
    public static final double WheelDiameterMeters = Units.feetToMeters(0.5);
    public static final double WheelCircumferenceMeters = WheelDiameterMeters * Math.PI;
    public static final double kDriveNEOPositionFactor = kGearboxRatio * kWheelCircumferenceFeet;
    public static final double kDriveNEOVelocityFactor = kDriveNEOPositionFactor / 60.0;

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
