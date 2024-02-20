package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

public class WestCoastDrive extends Module {
  private final CANSparkMax mLeftMaster = new CANSparkMax(REVLibCAN.L_MASTER_ID, REVLibCAN.MOTOR_TYPE);
  private final CANSparkMax mLeftFollower = new CANSparkMax(REVLibCAN.L_FOLLOWER_ID, REVLibCAN.MOTOR_TYPE);
  private final CANSparkMax mRightMaster = new CANSparkMax(REVLibCAN.R_MASTER_ID, REVLibCAN.MOTOR_TYPE);
  private final CANSparkMax mRightFollower = new CANSparkMax(REVLibCAN.R_FOLLOWER_ID, REVLibCAN.MOTOR_TYPE);

  private final RelativeEncoder mLeftEncoder = mLeftMaster.getEncoder();
  private final RelativeEncoder mRightEncoder = mLeftMaster.getEncoder();

  private final DifferentialDrive mDrive = new DifferentialDrive(mLeftMaster::set, mRightMaster::set);

  private final ADXRS450_Gyro mGyro = new ADXRS450_Gyro();

  private final DifferentialDriveOdometry mOdometry;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                mLeftMaster.setVoltage(volts.in(Volts));
                mRightMaster.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mLeftMaster.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mLeftEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(mLeftEncoder.getVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mRightMaster.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mRightEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(mRightEncoder.getVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));


  private static final WestCoastDrive instance = new WestCoastDrive();

  public static WestCoastDrive getInstance() {
    return instance;
  }

  /** Creates a new DriveSubsystem. */
  public WestCoastDrive() {
    SendableRegistry.addChild(mDrive, mLeftMaster);
    SendableRegistry.addChild(mDrive, mRightMaster);

    mLeftMaster.restoreFactoryDefaults();
    mLeftFollower.restoreFactoryDefaults();
    mRightMaster.restoreFactoryDefaults();
    mRightFollower.restoreFactoryDefaults();

    // Sets the distance per pulse for the encoders
    // mLeftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
    // mRightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
    mLeftEncoder.setPosition(0.0);
    mRightEncoder.setPosition(0.0);

    mLeftEncoder.setPositionConversionFactor(RobotMap.DriveConstants.kLinearDistanceConversionFactor);
    mRightEncoder.setPositionConversionFactor(RobotMap.DriveConstants.kLinearDistanceConversionFactor);
    mLeftEncoder.setVelocityConversionFactor(RobotMap.DriveConstants.kLinearDistanceConversionFactor / 60);
    mRightEncoder.setVelocityConversionFactor(RobotMap.DriveConstants.kLinearDistanceConversionFactor / 60);
    
    mLeftFollower.follow(mLeftMaster);
    mRightFollower.follow(mRightMaster);

    mRightMaster.setInverted(true);

    mGyro.reset();
    mGyro.calibrate();
    resetEncoders();

    mOdometry = new DifferentialDriveOdometry(
          mGyro.getRotation2d(), mLeftEncoder.getPosition(), mRightEncoder.getPosition());
    mOdometry.resetPosition(mGyro.getRotation2d(), mLeftEncoder.getPosition(), mRightEncoder.getPosition(), getPose());
    
    mLeftMaster.setSmartCurrentLimit(25);
    mLeftFollower.setSmartCurrentLimit(25);
    mRightMaster.setSmartCurrentLimit(25);
    mRightFollower.setSmartCurrentLimit(25);
    mDrive.setSafetyEnabled(true);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    mOdometry.update(
      mGyro.getRotation2d(), mLeftEncoder.getPosition(), mRightEncoder.getPosition());

    SmartDashboard.putNumber("Left Encoder Value Meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Value Meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro Heading", getHeading());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pPose) {
    resetEncoders();
    mOdometry.resetPosition(
        mGyro.getRotation2d(), mLeftEncoder.getPosition(), mRightEncoder.getPosition(), pPose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    mDrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double pLeftVolts, double pRightVolts) {
    mLeftMaster.setVoltage(pLeftVolts);
    mRightMaster.setVoltage(pRightVolts);
    mDrive.feed();
  }

  public void setCoastMode() {
    mLeftMaster.setIdleMode(IdleMode.kCoast);
    mLeftFollower.setIdleMode(IdleMode.kCoast);
    mRightMaster.setIdleMode(IdleMode.kCoast);
    mRightFollower.setIdleMode(IdleMode.kCoast);
  }

  public void setBreakMode() {
    mLeftMaster.setIdleMode(IdleMode.kBrake);
    mLeftFollower.setIdleMode(IdleMode.kBrake);
    mRightMaster.setIdleMode(IdleMode.kBrake);
    mRightFollower.setIdleMode(IdleMode.kBrake);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    mLeftEncoder.setPosition(0.0);
    mRightEncoder.setPosition(0.0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return mLeftEncoder;
  }

  public double getLeftEncoderPosition() {
    return -mLeftEncoder.getPosition();
  }

  public double getLeftEncoderVelocity() {
    return -mLeftEncoder.getVelocity();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return mRightEncoder;
  }

  public double getRightEncoderPosition() {
    return mRightEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return mRightEncoder.getVelocity();
  }

  public ADXRS450_Gyro getGyro() {
    return mGyro;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double pMaxOutput) {
    mDrive.setMaxOutput(pMaxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    mGyro.calibrate();
    mGyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return mGyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -mGyro.getRate();
  }

    /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> mDrive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void setVolts(double pLeftVolts, double pRightVolts) {
        // Safety check, only if the desired .set() value is less than one should it be
        // set to the motors
        if (Math.abs(pLeftVolts / 12) < 1 && Math.abs(pRightVolts / 12) < 1) {
                mLeftMaster.set(pLeftVolts / 12);
                mRightMaster.set(pRightVolts / 12);
        }
        // mTable.getEntry("volts").setNumber(leftVolts / 12);
  }
}