// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Modified by Avi Sharma
package frc.robot.subsystems;

import java.util.stream.Stream;

import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// import frc.common.lib.control.PIDController;

import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import frc.robot.DriveConstants;
import lombok.Getter;

/**
 * WestCoastDrive Subsystem.
 */
@SuppressWarnings("PMD.CommentSize")
public class WestCoastDrive extends Module {
        private final Field2d mField = new Field2d();

        private @Getter final ADAM adam = new ADAM(null);

        // Our 4 MOTORS
        private static CANSparkMax mLeftMaster = new CANSparkMax(REVLibCAN.L_MASTER_ID, REVLibCAN.MOTOR_TYPE),
                        mLeftFollower = new CANSparkMax(REVLibCAN.L_FOLLOWER_ID, REVLibCAN.MOTOR_TYPE);
        private static CANSparkMax mRightMaster = new CANSparkMax(REVLibCAN.R_MASTER_ID, REVLibCAN.MOTOR_TYPE),
                        mRightFollower = new CANSparkMax(REVLibCAN.R_FOLLOWER_ID, REVLibCAN.MOTOR_TYPE);

        private final DifferentialDrive m_drive = new DifferentialDrive(mLeftMaster::set, mRightMaster::set);
        private final DifferentialDriveOdometry mOdometry;

        // TODO: fix these values to actual port numbers
        private final RelativeEncoder mLeftEncoder = mLeftMaster.getEncoder(); // This is how you do it right
        private final RelativeEncoder mRightEncoder = mRightMaster.getEncoder();

        private ADXRS450_Gyro mGyro = new ADXRS450_Gyro();

        private static final WestCoastDrive instance = new WestCoastDrive();

        public double mCurrentDeg;

        /** Constructor for the Subsystem */
        public WestCoastDrive() {
                super();
                mTable = NetworkTableInstance.getDefault().getTable("drive");

                // Restore factory defaults for all motors
                mLeftMaster.restoreFactoryDefaults();
                mLeftFollower.restoreFactoryDefaults();
                mRightMaster.restoreFactoryDefaults();
                mRightFollower.restoreFactoryDefaults();

                // Set followers for each master
                mLeftFollower.follow(mLeftMaster);
                mRightFollower.follow(mRightMaster);

                // Set inversion for motors
                mLeftMaster.setInverted(false);
                mLeftFollower.setInverted(false);
                mRightMaster.setInverted(true);
                mRightFollower.setInverted(true);

                // Set idle mode to coast for all motors
                mLeftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
                mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
                mRightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
                mRightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);

                // Set smart current limit for all motors
                mLeftMaster.setSmartCurrentLimit(65);
                mLeftFollower.setSmartCurrentLimit(65);
                mRightMaster.setSmartCurrentLimit(65);
                mRightFollower.setSmartCurrentLimit(65);

                // Set closed-loop ramp rate for all motors
                mLeftMaster.setClosedLoopRampRate(1);
                mLeftFollower.setClosedLoopRampRate(1);
                mRightMaster.setClosedLoopRampRate(1);
                mRightFollower.setClosedLoopRampRate(1);

                // Set control frame period for all motors
                mLeftMaster.setControlFramePeriodMs(1);
                mLeftFollower.setControlFramePeriodMs(1);
                mRightMaster.setControlFramePeriodMs(1);
                mRightFollower.setControlFramePeriodMs(1);

                // TODO: FIX THIS BY USING ACTUAL VALUES
                setDistancePerPulse(mLeftEncoder, DriveConstants.kEncoderDistancePerPulse);
                setDistancePerPulse(mRightEncoder, DriveConstants.kEncoderDistancePerPulse);

                resetEncoders();
                mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d(), getDistance(mLeftEncoder),
                                getDistance(mRightEncoder));

                Stream.of(mLeftMaster, mLeftFollower, mRightMaster, mRightFollower)
                                .forEach(CANSparkMax::burnFlash);
                SmartDashboard.putData("Field", mField);
        }

        @Override
        public void periodic() { // This method will be called once per scheduler run (usually, once every 20
                                 // ms),
                runTest(() -> {
                        mField.setRobotPose(getPose());
                        REVLibCAN.logFaults(Stream.of(mLeftMaster, mLeftFollower, mRightMaster, mRightFollower));

                        var mGyroAngle = mGyro.getRotation2d();
                        mOdometry.update(mGyroAngle, getDistance(mLeftEncoder),
                                        getDistance(mRightEncoder));

                        leftMaster.setDouble(mLeftMaster.get());
                        leftFollower.setDouble(mLeftFollower.get());
                        rightMaster.setDouble(mRightMaster.get());
                        rightFollower.setDouble(mRightFollower.get());
                        gyro.setDouble(mGyro.getRotation2d().getDegrees());
                        // ... Other periodic tasks
                });
        }

        public Pose2d getPose() {
                return mOdometry.getPoseMeters();
        }

        public DifferentialDriveWheelSpeeds getWheelSpeeds() {
                return new DifferentialDriveWheelSpeeds(getRate(mLeftEncoder), getRate(mRightEncoder));
        }

        public void resetOdometry(Pose2d pose) {
                resetEncoders();
                mOdometry.resetPosition(
                                mGyro.getRotation2d(), getDistance(mLeftEncoder), getDistance(mRightEncoder), pose);
        }

        public void setMotorSpeed(final double leftSpeed, final double rightSpeed) {
                mLeftMaster.set(leftSpeed);
                mRightMaster.set(rightSpeed);
        }

        public double getMotorSpeed() {
                return mLeftMaster.get();
        }

        public void tankDriveVolts(double leftVolts, double rightVolts) {
                mLeftMaster.setVoltage(leftVolts);
                mRightMaster.setVoltage(rightVolts);
                m_drive.feed();
        }

        public void resetEncoders() {
                reset(mLeftEncoder);
                reset(mRightEncoder);
        }

        public double getAverageEncoderDistance() {
                return (getDistance(mLeftEncoder) + getDistance(mRightEncoder)) / 2.0;
        }

        public RelativeEncoder getLeftEncoder() {
                return mLeftEncoder;
        }

        public RelativeEncoder getRightEncoder() {
                return mRightEncoder;
        }

        public void setMaxOutput(double maxOutput) {
                m_drive.setMaxOutput(maxOutput);
        }

        public void zeroHeading() {
                mGyro.reset();
        }

        public double getHeading() {
                return mGyro.getRotation2d().getDegrees();
        }

        public double getTurnRate() {
                return -mGyro.getRate();
        }

        public static WestCoastDrive getInstance() {
                return instance;
        }

        public void debugSubsystem() {
                runTest(() -> periodic());
        }

        public void runTest(final Runnable code) {
                try {
                        code.run();
                } catch (Exception e) {
                        adam.uncaughtException(Thread.currentThread(), e);
                }
        }

        // SHUFFLEBOARD
        private NetworkTable mTable;
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

        // Tried creating Encoder class methods from docs and videos, unsure if they are
        // correct
        public double getDistance(RelativeEncoder encoder) {
                return encoder.getPosition() * DriveConstants.kEncoderDistancePerPulse;
        }

        public void setDistancePerPulse(RelativeEncoder encoder, double distancePerPulse) {
                encoder.setPositionConversionFactor(1.0 / distancePerPulse);
        }

        public double getRate(RelativeEncoder encoder) {
                return encoder.getVelocity() * DriveConstants.kEncoderDistancePerPulse;
        }

        public void reset(RelativeEncoder encoder) {
                encoder.setPosition(0);
        }
}
