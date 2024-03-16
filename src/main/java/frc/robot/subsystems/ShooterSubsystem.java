package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DrivetrainConstants;;

public class ShooterSubsystem extends PIDSubsystem {
  private final PWMSparkMax m_frontRMotor = new PWMSparkMax(DrivetrainConstants.kRightFrontID);
  private final PWMSparkMax m_frontLMotor = new PWMSparkMax(DrivetrainConstants.kLeftFrontID);
  private final PWMSparkMax m_backRMotor = new PWMSparkMax(DrivetrainConstants.kRightRearID);
  private final PWMSparkMax m_backLMotor = new PWMSparkMax(DrivetrainConstants.kLeftRearID);
  /*private final Encoder m_shooterEncoder =
      new Encoder(
          LauncherConstants.kEncoderPorts[0],
          ShooterConstants.kEncoderPorts[1],
          ShooterConstants.kEncoderReversed);*/
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          DrivetrainConstants.ksVolts, DrivetrainConstants.kVVoltSecondsPerRotation);

  /** The shooter subsystem for the robot. */
  public ShooterSubsystem() {
    super(new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD));
    getController().setTolerance(DrivetrainConstants.kShooterToleranceRPS);
    m_shooterEncoder.setDistancePerPulse(DrivetrainConstants.kEncoderDistancePerPulse);
    setSetpoint(DrivetrainConstants.kShooterTargetRPS);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    //return m_shooterEncoder.getRate();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void runFeeder() {
    //m_feederMotor.set(ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    m_feederMotor.set(0);
  }
}