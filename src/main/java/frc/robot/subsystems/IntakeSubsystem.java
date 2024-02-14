// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.stream.Stream;

import javax.sound.sampled.Port;

import org.apache.logging.log4j.util.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.vendors.firstparties.ABC;
import frc.robot.hardware.vendors.thirdparties.EGyro;
import frc.robot.hardware.vendors.thirdparties.LL3.ELimelightData;
import frc.robot.hardware.vendors.thirdparties.ctre.CTREPigeon;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.common.config.Settings;
// import frc.common.lib.control.PIDController;
import frc.common.lib.control.ProfileGains;
import frc.common.types.EMatchMode;
import frc.robot.ADAM;
import frc.robot.Enums;
import frc.robot.Robot;
import frc.robot.InputMap.EDriveData;
import frc.robot.hardware.ECommonNeutralMode;
import frc.robot.hardware.HardwareUtils;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;

/**
 * WestCoastDrive Subsystem.
 */
@SuppressWarnings("PMD.CommentSize")
public class IntakeSubsystem extends Module {
        private @Getter final ADAM adam = new ADAM(null);

        // Creates two CANSparkMax motors, inheriting physical constants from the
        // {@link#REVLibCAN} helper class.
        private static CANSparkMax mShooterMain = new CANSparkMax(REVLibCAN.SHOOTER1_ID, REVLibCAN.MOTOR_TYPE),
                        mShooterFollower = new CANSparkMax(REVLibCAN.SHOOTER2_ID, REVLibCAN.MOTOR_TYPE);

        private @Getter RelativeEncoder mShooterEncoder = mShooterMain.getEncoder();

        //private SparkPIDController mLeftCtrl, mRightCtrl;
        // private PIDController mTurnToDegreePID, mLeftPositionPID, mRightPositionPID,
        // mTargetLockPID;
        //private ADXRS450_Gyro mGyro = new ADXRS450_Gyro();

        private NetworkTable mTable;
        private final Field2d mField = new Field2d();
        private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        private GenericEntry shooterMaster = tab.add("Shooter Master", 0)
                        .getEntry();


        private static final WestCoastDrive instance = new WestCoastDrive();

        public static WestCoastDrive getInstance() {
                return instance;
        }

        public double mCurrentDeg;

        /** Constructor for the Subsystem */
        public IntakeSubsystem() {
                super();
                mTable = NetworkTableInstance.getDefault().getTable("drive");
                Stream.of(mShooterMain, mShooterFollower).forEach(CANSparkMax::restoreFactoryDefaults);
                final Map<CANSparkMax, CANSparkMax> masterFollowerMap = Map.of(mShooterMain, mShooterFollower);
                masterFollowerMap.forEach((master, follower) -> follower.follow(master));
                Stream.of(mShooterMain).forEach(motor -> motor.setInverted(false));
                Stream.of(mShooterFollower).forEach(motor -> motor.setInverted(true));
                Stream.of(mShooterMain, mShooterFollower).forEach(motor -> motor.setIdleMode(CANSparkMax.IdleMode.kCoast));
                Stream.of(mShooterMain, mShooterFollower).forEach(motor -> motor.setSmartCurrentLimit(65));
                Stream.of(mShooterMain, mShooterFollower).forEach(motor -> motor.setClosedLoopRampRate(1));
                // Stream.of(mLeftMaster, mLeftFollower, mRightMaster, mRightFollower)
                // .forEach(motor -> motor.setOpenLoopRampRate(0.5));
                Stream.of(mShooterMain, mShooterFollower).forEach(motor -> motor.setControlFramePeriodMs(1));
                mShooterEncoder = mShooterMain.getEncoder();

                Stream.of(mShooterMain, mShooterFollower).forEach(CANSparkMax::burnFlash);
                /*SmartDashboard.putData("Field", mField);
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
                );*/
        }

        /*double shooterCountsPerRevolution = mShooterEncoder.getCountsPerRevolution();

        //double shooterEncoderPosition = mShooterEncoder.getPosition();

        //double leftDistanceTraveled = (leftEncoderPosition / leftCountsPerRevolution)
                        * (ABC.feet_to_meters(kWheelDiameterFeet) * Math.PI);
        double rightDistanceTraveled = (rightEncoderPosition / rightCountsPerRevolution)
                        * (ABC.feet_to_meters(kWheelDiameterFeet) * Math.PI);
        private DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d(),
                        leftDistanceTraveled, rightDistanceTraveled);*/

        @Override
        public void periodic() { // This method will be called once per scheduler run (usually, once every 20
                                 // ms),
                runTest(() -> {
                        //mField.setRobotPose(getPose());
                        REVLibCAN.logFaults(Stream.of(mShooterMain, mShooterFollower));

                        //var mGyroAngle = mGyro.getRotation2d();
                        //mOdometry.update(mGyroAngle, leftDistanceTraveled, rightDistanceTraveled);

                        //shooterMaster.setDouble(mLeftMaster.get());
                        //leftFollower.setDouble(mLeftFollower.get());
                        //rightMaster.setDouble(mRightMaster.get());
                        //rightFollower.setDouble(mRightFollower.get());
                        //gyro.setDouble(mGyro.getRotation2d().getDegrees());
                        // ... Other periodic tasks
                });
        }

        /*public void drive(ChassisSpeeds pRobotRelativeSpeeds) {
                // Creating my kinematics object: track width of around 1.7 feet
                DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(ABC.feet_to_meters(kTrackWidthFeet));

                // Convert to wheel speeds
                DifferentialDriveWheelSpeeds mWheelSpeeds = mKinematics.toWheelSpeeds(pRobotRelativeSpeeds);

                // Left velocity
                double mLeftVelocity = mWheelSpeeds.leftMetersPerSecond;

                // Right velocity
                double mRightVelocity = mWheelSpeeds.rightMetersPerSecond;

                setMotorSpeed(mLeftVelocity, mRightVelocity);
        }*/

        public double getCurrentSpeeds() {
                // Create a kinematics object with a track width of around 1.7 feet
                //DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(ABC.feet_to_meters(kTrackWidthFeet));
                // Get the current left and right wheel speeds from the encoders
                double shooterSpeed = mShooterEncoder.getVelocity();
                // Create a wheel speeds object with the encoder values
                //DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftSpeed * kRpmToMpsFactor,
                                //rightSpeed * kRpmToMpsFactor);
                // Convert the wheel speeds to chassis speeds
                //ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
                //return chassisSpeeds;
                return shooterSpeed;
        }

        /*public Pose2d getPose() {
                return mOdometry.getPoseMeters();
        }

        public void resetPose(Pose2d pose) {
                pose = mOdometry.getPoseMeters();
                mOdometry.resetPosition(mGyro.getRotation2d(), leftDistanceTraveled, rightDistanceTraveled,
                                pose);
        }*/

        public void setMotorSpeed(final double shootSpeed) {
                mShooterMain.set(shootSpeed);
        }

        public double getMotorSpeed() {
                // Getting motor speed using the ".get()" method from the CANSparkMax class
                return mShooterMain.get();
        }

        /*public void setThrottlePct(double throttle) {
                mShooterMain.set(throttle);
                mTable.getEntry("THROTTLE PCT").setNumber(throttle);
        }

        public double getGyroRollDeg() {
                // mTable.getEntry("ROLL DEG").setNumber(mGyro.get.getDegrees());
                // return mGyro.getRoll().getDegrees();
                // TODO: Fix ADXRS450_Gyro Implementations
                return 0.0;
        }*/

        /**
         * Executes a custom method, running it within a testing environment.
         *
         * @see #runTest(Runnable)
         */
        public void reset() {
                runTest(() -> {
                        Stream.of(mShooterEncoder)
                                        .forEach(encoder -> encoder.setPosition(0.0));
                        Stream.of(mShooterMain)
                                        .forEach(motor -> motor.set(0.0));
                        Stream.of(mShooterMain)
                                        .forEach(motor -> motor.stopMotor());
                });
        }

        /*public void setVolts(double leftVolts, double rightVolts) {
                // Safety check, only if the desired .set() value is less than one should it be
                // set to the motors
                if (Math.abs(leftVolts / 12) < 1 && Math.abs(rightVolts / 12) < 1) {
                        mLeftMaster.set(leftVolts / 12);
                        mRightMaster.set(rightVolts / 12);
                }
                mTable.getEntry("volts").setNumber(leftVolts / 12);
        }

        public void modeInit(EMatchMode pMode) {
                mGyro.reset();
                reset();
                // mCurrentDeg = 80;
                if (pMode == EMatchMode.AUTONOMOUS) {
                        // resetPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
                        mLeftMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
                        mRightMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
                        mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
                        mRightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
                } else {
                        mLeftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
                        mRightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
                        mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
                        mRightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
                }
        }

        @Override
        public void readInputs() {
                // mGyro.;
                // db.drivetrain.set(EDriveData.ACTUAL_HEADING_RADIANS,
                // -mGyro.getHeading().getRadians());
                // db.drivetrain.set(EDriveData.ACTUAL_HEADING_DEGREES,
                // -mGyro.getHeading().getDegrees());
                db.drivetrain.set(EDriveData.LEFT_VOLTAGE, mLeftMaster.getVoltageCompensationNominalVoltage());
                db.drivetrain.set(EDriveData.RIGHT_VOLTAGE, mRightMaster.getVoltageCompensationNominalVoltage());
                db.drivetrain.set(EDriveData.LEFT_CURRENT, mLeftMaster.getOutputCurrent());
                db.drivetrain.set(EDriveData.RIGHT_CURRENT, mRightMaster.getOutputCurrent());
                db.drivetrain.set(EDriveData.L_ACTUAL_POS_FT, mLeftEncoder.getPosition() * kDriveNEOPositionFactor);
                db.drivetrain.set(EDriveData.L_ACTUAL_VEL_FT_s, mLeftEncoder.getVelocity() * kDriveNEOVelocityFactor);
                db.drivetrain.set(EDriveData.R_ACTUAL_VEL_RPM, mRightEncoder.getVelocity() * kGearboxRatio);
                db.drivetrain.set(EDriveData.L_ACTUAL_VEL_RPM, mLeftEncoder.getVelocity() * kGearboxRatio);
                db.drivetrain.set(EDriveData.R_ACTUAL_POS_FT, mRightEncoder.getPosition() * kDriveNEOPositionFactor);
                db.drivetrain.set(EDriveData.R_ACTUAL_VEL_FT_s, mRightEncoder.getVelocity() * kDriveNEOVelocityFactor);
                // db.imu.set(EGyro.ACCEL_X, mGyro.getAccelX());
                // db.imu.set(EGyro.ACCEL_Y, mGyro.getAccelY());
                // db.imu.set(EGyro.PITCH_DEGREES, mGyro.getPitch().getDegrees());
                // db.imu.set(EGyro.ROLL_DEGREES, mGyro.getRoll().getDegrees());
                // db.imu.set(EGyro.YAW_DEGREES, -mGyro.getYaw().getDegrees());
                // db.imu.set(EGyro.YAW_OMEGA_DEGREES, -mGyro.getYawRate().getDegrees());
                db.drivetrain.set(EDriveData.X_ACTUAL_ODOMETRY_METERS, mOdometry.getPoseMeters().getX());
                db.drivetrain.set(EDriveData.Y_ACTUAL_ODOMETRY_METERS, mOdometry.getPoseMeters().getY());
                // mOdometry.update(mGyro.getHeading(),
                // ABC.feet_to_meters(db.drivetrain.get(EDriveData.L_ACTUAL_POS_FT)),
                // ABC.feet_to_meters(db.drivetrain.get(EDriveData.R_ACTUAL_POS_FT)));
                Robot.FIELD.setRobotPose(mOdometry.getPoseMeters());
        }

        @Override
        public void setOutputs() {
                Enums.EDriveState state = db.drivetrain.get(EDriveData.STATE, Enums.EDriveState.class);
                double throttle = db.drivetrain.safeGet(EDriveData.DESIRED_THROTTLE_PCT, 0.0);
                double turn = db.drivetrain.safeGet(EDriveData.DESIRED_TURN_PCT, 0.0);
                double left = throttle + turn;
                double right = throttle - turn;
                ECommonNeutralMode neutralMode = db.drivetrain.get(EDriveData.NEUTRAL_MODE, ECommonNeutralMode.class);
                if (state == null)
                        return;
                switch (state) {
                        case RESET:
                                mGyro.reset();
                                ;
                                reset();
                                break;
                        case RESET_ODOMETRY:
                                double x = db.drivetrain.get(EDriveData.X_DESIRED_ODOMETRY_METERS);
                                double y = db.drivetrain.get(EDriveData.X_DESIRED_ODOMETRY_METERS);
                                mGyro.reset();
                                // resetPose(new Pose2d(x, y, new Rotation2d(-mGyro.getYaw().getRadians())));
                                break;
                        case PERCENT_OUTPUT:
                                if (db.limelight.isSet(ELimelightData.TARGET_ID)) {
                                        // mTable.getEntry("AM I tracking idk").setString("Yippie");
                                        double targetLockOutput = 0;
                                        if (db.limelight.isSet(ELimelightData.TV)) {
                                                // targetLockOutput = mTargetLockPID.calculate(
                                                // -db.limelight.get(ELimelightData.TX),
                                                // clock.deltaTime());
                                                // turn = targetLockOutput;
                                        }
                                        turn += 0.1 * Math.signum(turn);
                                        turn *= (1 / (1 - throttle)) * 0.5;
                                }
                                mLeftMaster.set(throttle + turn);
                                mRightMaster.set(throttle - turn);
                                break;
                        case VELOCITY:
                                mLeftCtrl.setReference(left * kMaxVelocityRPM, CANSparkMax.ControlType.kVelocity,
                                                VELOCITY_PID_SLOT, 0);
                                mRightCtrl.setReference(right * kMaxVelocityRPM, CANSparkMax.ControlType.kVelocity,
                                                VELOCITY_PID_SLOT, 0);
                                break;
                        case BREAK:
                                mLeftMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
                                mRightMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
                                mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
                                mRightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
                }
        }*/

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