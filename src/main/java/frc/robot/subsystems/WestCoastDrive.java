// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.vendors.firstparties.ABC;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.common.config.Settings;
// import frc.common.lib.control.PIDController;
import frc.common.lib.control.ProfileGains;
import frc.robot.ADAM;
import frc.robot.hardware.HardwareUtils;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;

/**
 * WestCoastDrive Subsystem.
 */
@SuppressWarnings("PMD.CommentSize")
public class WestCoastDrive extends Module {
        private @Getter final ADAM adam = new ADAM(null);

        // Creates two CANSparkMax motors, inheriting physical constants from the
        // {@link#REVLibCAN} helper class.
        private CANSparkMax mLeftMaster = new CANSparkMax(REVLibCAN.L_MASTER_ID, REVLibCAN.MOTOR_TYPE);
        private CANSparkMax mLeftFollower = new CANSparkMax(REVLibCAN.L_FOLLOWER_ID, REVLibCAN.MOTOR_TYPE);

        // Creates two CANSparkMax motors, inheriting physical constants from the
        // {@link#REVLibCAN} helper class.
        private CANSparkMax mRightMaster = new CANSparkMax(REVLibCAN.R_MASTER_ID, REVLibCAN.MOTOR_TYPE);
        private CANSparkMax mRightFollower = new CANSparkMax(REVLibCAN.R_FOLLOWER_ID, REVLibCAN.MOTOR_TYPE);

        private @Getter RelativeEncoder mLeftEncoder;
        private @Getter RelativeEncoder mRightEncoder;
        private ADXRS450_Gyro mGyro = new ADXRS450_Gyro();

        private DifferentialDrive mDifferentialDrive = new DifferentialDrive(mRightMaster, mLeftMaster);

        private DifferentialDriveOdometry mOdometry;
        private DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(
                        ABC.feet_to_meters(kTrackWidthFeet));

        ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);
        DifferentialDriveWheelSpeeds mWheelSpeeds = mKinematics.toWheelSpeeds(mChassisSpeeds);

        double leftVelocity = mWheelSpeeds.leftMetersPerSecond;
        double rightVelocity = mWheelSpeeds.rightMetersPerSecond;

        private SparkPIDController mLeftCtrl, mRightCtrl;

        private NetworkTable mTable;
        private final Field2d mField = new Field2d();
        private ShuffleboardTab tab = Shuffleboard.getTab("West Coast Drive");
        private GenericEntry leftMaster = tab.add("Left Master", 0)
                        .getEntry();
        private GenericEntry leftFollower = tab.add("Left Follower", 0)
                        .getEntry();
        private GenericEntry rightMaster = tab.add("Right Master", 0)
                        .getEntry();
        private GenericEntry rightFollower = tab.add("Right Follower", 0)
                        .getEntry();
        private GenericEntry gyro = tab.add("Gyro", 0).getEntry();

        // ========================================
        // DO NOT MODIFY THESE PHYSICAL CONSTANTS
        // ========================================
        public static final double kGearboxRatio = 8.46; // rotated 8.46 times, rotates gear that spins the wheel once
        public static final double kWheelDiameterInches = 6;
        public static final double kWheelDiameterFeet = 0.5;
        public static final double kWheelCircumferenceInches = kWheelDiameterInches * Math.PI;
        public static final double kWheelCircumferenceFeet = kWheelDiameterFeet * Math.PI;

        public static final double kReductionRatio = 0.01;
        public static final double kConversionFactor = kWheelCircumferenceInches * kReductionRatio;
        public static final double kDriveNEOPositionFactor = kGearboxRatio * kWheelCircumferenceFeet;
        public static final double kDriveNEOVelocityFactor = kDriveNEOPositionFactor / 60.0;
        public static final double kMaxVelocityRPM = 5676;
        public static final double kPulsesPerRotation = 256.0;
        public static final double kCurrentLimitAmps = 60.0;
        public static final double kTrackWidthFeet = 20.875 / 12;
        public static final double kRpmToMpsFactor = (ABC.feet_to_meters(kWheelCircumferenceFeet) / kGearboxRatio) * 1
                        / 60;
        public static final double kMaxMpsVelocity = kRpmToMpsFactor * kMaxVelocityRPM;
        public static final int kMaxLimelightFOV = 22;

        public static final int kEncoderCPR = 1024;
        public static final double kEncoderDistancePerPulse = (ABC.feet_to_meters(kTrackWidthFeet) * Math.PI)
                        / (double) kEncoderCPR;
        // ========================================
        // DO NOT MODIFY THESE PID CONSTANTS
        // ========================================
        private static final int VELOCITY_PID_SLOT = 1;
        private static final int SMART_MOTION_PID_SLOT = 2;
        // private static final E NEUTRAL_MODE = null;
        // TODO change the smart motion gains once we have tuned it
        public static ProfileGains kSmartMotionGains = new ProfileGains()
                        .p(0.25)
                        .f(0.00015)
                        .maxVelocity(kMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
                        .maxAccel(5676d)
                        .slot(SMART_MOTION_PID_SLOT)
                        .velocityConversion(kDriveNEOPositionFactor);
        public static ProfileGains kPositionGains = new ProfileGains()
                        .maxVelocity(kMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier * 12)
                        .maxAccel(31 * 12d) // just under 1g
                        .p(0.0275)
                        .tolerance(0.2);
        public static ProfileGains kVelocityGains = new ProfileGains()
                        .f(0.00015)
                        .p(0.00000025)
                        .maxVelocity(kMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
                        .slot(VELOCITY_PID_SLOT)
                        .velocityConversion(kDriveNEOVelocityFactor);
        public static ProfileGains kTurnToProfileGains = new ProfileGains().p(0.02).f(0.1);
        // Old value is 0.0075
        public static ProfileGains kTargetAngleLockGains = new ProfileGains().p(0.01);

        // ========================================
        // DO NOT MODIFY THESE OTHER CONSTANTS
        // ========================================
        public static double kTurnSensitivity = 0.85;

        private static final WestCoastDrive instance = new WestCoastDrive();

        public static WestCoastDrive getInstance() {
                return instance;
        }

        public double mCurrentDeg;

        /** Constructor for the Subsystem */
        public WestCoastDrive() {
                super();
                mTable = NetworkTableInstance.getDefault().getTable("drive");

                mLeftMaster.restoreFactoryDefaults();
                mLeftFollower.restoreFactoryDefaults();
                mRightMaster.restoreFactoryDefaults();
                mRightFollower.restoreFactoryDefaults();

                mRightMaster.setInverted(false);
                mRightFollower.setInverted(false);

                mRightMaster.setInverted(true);
                mRightFollower.setInverted(true);

                mLeftFollower.follow(mLeftMaster);
                mRightFollower.follow(mRightMaster);

                mGyro.calibrate();

                mDifferentialDrive.setSafetyEnabled(false);

                mLeftEncoder = mLeftMaster.getEncoder();
                mRightEncoder = mRightMaster.getEncoder();
                mLeftEncoder.setPositionConversionFactor(kEncoderDistancePerPulse);
                mRightEncoder.setPositionConversionFactor(kEncoderDistancePerPulse);
                mLeftEncoder.setVelocityConversionFactor(kEncoderDistancePerPulse / 60);
                mRightEncoder.setVelocityConversionFactor(kEncoderDistancePerPulse / 60);

                Stream.of(mLeftMaster, mLeftFollower, mRightMaster, mRightFollower)
                                .forEach(motor -> motor.setIdleMode(CANSparkMax.IdleMode.kCoast));
                Stream.of(mLeftMaster, mLeftFollower, mRightMaster, mRightFollower)
                                .forEach(motor -> motor.setSmartCurrentLimit(65));
                Stream.of(mLeftMaster, mLeftFollower, mRightMaster, mRightFollower)
                                .forEach(motor -> motor.setClosedLoopRampRate(1));
                // Stream.of(mLeftMaster, mLeftFollower, mRightMaster, mRightFollower)
                //                 .forEach(motor -> motor.setOpenLoopRampRate(0.5));
                Stream.of(mLeftMaster, mLeftFollower, mRightMaster, mRightFollower)
                                .forEach(motor -> motor.setControlFramePeriodMs(1));

                resetEncoders();

                mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(mGyro.getAngle()),
                                mLeftEncoder.getPosition(), mRightEncoder.getPosition());

                mRightCtrl = mRightMaster.getPIDController();
                mLeftCtrl = mLeftMaster.getPIDController();
                mRightCtrl.setOutputRange(-kMaxVelocityRPM, kMaxVelocityRPM);
                mLeftCtrl.setOutputRange(-kMaxVelocityRPM, kMaxVelocityRPM);

                HardwareUtils.setGains(mLeftCtrl, kVelocityGains);
                HardwareUtils.setGains(mRightCtrl, kVelocityGains);
                HardwareUtils.setGains(mLeftCtrl, kSmartMotionGains);
                HardwareUtils.setGains(mRightCtrl, kSmartMotionGains);

                Stream.of(mLeftMaster, mLeftFollower, mRightMaster, mRightFollower)
                                .forEach(CANSparkMax::burnFlash);
                SmartDashboard.putData("Field", mField);
                AutoBuilder.configureRamsete(
                                this::getPose, // Robot pose supplier
                                this::resetPose, // Method to reset odometry (will be called if your auto has a starting
                                                 // pose)
                                this::getCurrentSpeeds, // Current ChassisSpeeds supplier
                                this::drive, // Method that will drive the robot given ChassisSpeeds
                                new ReplanningConfig(), // Default path replanning config. See the API for the options
                                                        // here
                                () -> {
                                        // Boolean supplier that controls when the path will be mirrored for the red
                                        // alliance
                                        // This will flip the path being followed to the red side of the field.
                                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                                        var alliance = DriverStation.getAlliance();
                                        if (alliance.isPresent()) {
                                                return alliance.get() == DriverStation.Alliance.Red;
                                        }
                                        return false;
                                },
                                this // Reference to this subsystem to set requirements
                );
        }

        @Override
        public void periodic() { // This method will be called once per scheduler run (usually, once every 20
                                 // ms),
                runTest(() -> {
                        mField.setRobotPose(getPose());
                        REVLibCAN.logFaults(Stream.of(mLeftMaster, mLeftFollower, mRightMaster, mRightFollower));
                        // sets the Odometry with the current information.
                        mOdometry.update(Rotation2d.fromDegrees(mGyro.getAngle()), mLeftEncoder.getPosition(),
                                        mRightEncoder.getPosition());

                        leftMaster.setDouble(mLeftMaster.get());
                        leftFollower.setDouble(mLeftFollower.get());
                        rightMaster.setDouble(mRightMaster.get());
                        rightFollower.setDouble(mRightFollower.get());
                        gyro.setDouble(mGyro.getRotation2d().getDegrees());
                        // ... Other periodic tasks
                });
        }

        public void drive(ChassisSpeeds pChassisSpeeds) {
                // mDifferentialDrive.feed();
                // mLeftPIDController.setSetpoint(mChassisSpeeds.vxMetersPerSecond);
                // mRightPIDController.setSetpoint(mChassisSpeeds.omegaRadiansPerSecond);
                
                // mDifferentialDrive.arcadeDrive(0.5, 0.5);
        }

        public ChassisSpeeds getCurrentSpeeds() {
                return mKinematics.toChassisSpeeds(mWheelSpeeds);
        }

        public void resetEncoders() {
                mLeftEncoder.setPosition(0);
                mRightEncoder.setPosition(0);
        }

        public Pose2d getPose() {
                return mOdometry.getPoseMeters();

        }

        public void resetPose(Pose2d pose) {
                resetEncoders();
                mOdometry.resetPosition(Rotation2d.fromDegrees(mGyro.getAngle()), mLeftEncoder.getPosition(),
                                mRightEncoder.getPosition(), pose);
        }

        public void setMotorSpeed(double leftSpeed, double rightSpeed) {
                // set motor speed is slowed for testing
                
                mLeftMaster.set(leftSpeed);
                mRightMaster.set(rightSpeed);
        }

        public double getMotorSpeed() {
                // Getting motor speed using the ".get()" method from the CANSparkMax class
                return mLeftMaster.get();
        }

        public void setThrottlePct(double throttle) {
                mRightMaster.set(throttle);
                mLeftMaster.set(throttle);
                mTable.getEntry("THROTTLE PCT").setNumber(throttle);
        }

        public double getGyroRollDeg() {
                // mTable.getEntry("ROLL DEG").setNumber(mGyro.get.getDegrees());
                // return mGyro.getRoll().getDegrees();
                // TODO: Fix ADXRS450_Gyro Implementations
                return 0.0;
        }

        /**
         * Executes a custom method, running it within a testing environment.
         *
         * @see #runTest(Runnable)
         */
        public void reset() {
                runTest(() -> {
                        Stream.of(mLeftEncoder, mRightEncoder)
                                        .forEach(encoder -> encoder.setPosition(0.0));
                        Stream.of(mLeftMaster, mRightMaster)
                                        .forEach(motor -> motor.set(0.0));
                        Stream.of(mLeftMaster, mRightMaster)
                                        .forEach(motor -> motor.stopMotor());
                });
        }

        public void setVolts(double leftVolts, double rightVolts) {
                // Safety check, only if the desired .set() value is less than one should it be
                // set to the motors
                if (Math.abs(leftVolts / 12) < 1 && Math.abs(rightVolts / 12) < 1) {
                        mLeftMaster.set(leftVolts / 12);
                        mRightMaster.set(rightVolts / 12);
                }
                mTable.getEntry("volts").setNumber(leftVolts / 12);
        }

        /**
         * Executes custom testing and validation methods in a controlled environment.
         * Any exceptions thrown during execution are caught and logged.
         *
         * @see #runTest(Runnable)
         */
        public void debugSubsystem() {
                runTest(() -> periodic());
                runTest(() -> reset());
        }

        /**
         * Runs the code as a task; if an exception occurs, it is caught, and an
         * uncaught exception is passed to the default handler for the current thread.
         *
         * @param code The runnable task to be executed.
         */
        public void runTest(final Runnable code) {
                try {
                        code.run();
                } catch (Exception e) {
                        adam.uncaughtException(Thread.currentThread(), e);
                }
        }
}
