// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.commands.Climb;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rotator;
import frc.robot.subsystems.DriveSubsystem;
import lombok.Getter;

import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("PMD.CommentSize")
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    public static final DriveSubsystem mDrive = DriveSubsystem.getInstance();
    private final CANLauncher mLauncher = new CANLauncher();
    public final static Rotator mRotator = new Rotator();
    private final Intake mIntake = new Intake();
    private final Climber mClimber = new Climber();
    private final Climb mClimb;

    public final TalonRotate mRotate;
    private final AngleShooter mAngleShooter;

    private final double kP = 0.012;

    public final static CommandXboxController mDriverController = new CommandXboxController(1); // 1 is the USB Port to
                                                                                                // be used as indicated

    // on the Driver Station
    public final static Joystick mOperatorController = new Joystick(2); // 2 is the USB Port to be used as indicated on
                                                                        // the Driver Station

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Command Initialization
        mAngleShooter = new AngleShooter(mLauncher);

        mDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> mDrive.arcadeDrive(
                                -mDriverController.getLeftY(), -mDriverController.getRightX()),
                        mDrive));

        mRotate = new TalonRotate(mRotator);
        mRotator.setDefaultCommand(mRotate);

        mClimb = new Climb(mClimber);
        mClimber.setDefaultCommand(mClimb);

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings.
     * Triggers can be created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the named factories in
     * {@link CommandGenericHID}'s subclasses
     * for {@link CommandXboxController
     * Xbox}/{@link CommandPS4Controller PS4}
     * controllers or {@link CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        new JoystickButton(mOperatorController, 2)
                .whileTrue(
                        new PrepareLaunch(mLauncher)
                                .withTimeout(1)
                                .andThen(new LaunchNote(mLauncher))
                                .handleInterrupt(() -> mLauncher.stop()));

        new JoystickButton(mOperatorController, 3).whileTrue(mLauncher.feedlaunchWheel())
                .onFalse(new InstantCommand(mLauncher::stop));

        new JoystickButton(mOperatorController, 4)
                .whileTrue(
                        new PositionControl(mRotator)
                                .handleInterrupt(() -> mRotator.stop()));

        new JoystickButton(mOperatorController, 5)
                .whileTrue(
                        new TalonFeed(mIntake)
                                .handleInterrupt(() -> mIntake.stop()));

        new JoystickButton(mOperatorController, 6)
                .whileTrue(
                        new TalonShoot(mIntake)
                                .handleInterrupt(() -> mIntake.stop()));

        new JoystickButton(mOperatorController, 1).whileTrue(new AngleShooter(mLauncher).handleInterrupt(() -> mLauncher.stop()));

        //new JoystickButton(mOperatorController, 9).onTrue(new TurnToAngleProfiled(90, mDrive).withTimeout(2)
                //.andThen(Commands.runOnce(() -> mDrive.resetOdometry(new Pose2d()))));
        //new JoystickButton(mOperatorController, 10).onTrue(new TurnToAngleProfiled(-90, mDrive).withTimeout(2)
                //.andThen(Commands.runOnce(() -> mDrive.resetOdometry(new Pose2d()))));
    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() { // the code needs to do stuff
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.DrivetrainConstants.ksVolts,
                        Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
                        Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DrivetrainConstants.kDriveKinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.DrivetrainConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        RamseteCommand ramseteCommand = new RamseteCommand(
                Robot.toNoteTraj,
                mDrive::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        Constants.DrivetrainConstants.ksVolts,
                        Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
                        Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DrivetrainConstants.kDriveKinematics,
                mDrive::getWheelSpeeds,
                new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                mDrive::tankDriveVolts,
                mDrive);

        RamseteCommand ramseteCommand2 = new RamseteCommand(
                Robot.toSpeakerTraj,
                mDrive::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        Constants.DrivetrainConstants.ksVolts,
                        Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
                        Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DrivetrainConstants.kDriveKinematics,
                mDrive::getWheelSpeeds,
                new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                mDrive::tankDriveVolts,
                mDrive);

        // Reset odometry to the initial pose of the trajectory, run path following
        // command, then stop at the end.
        return Commands.runOnce(() -> mDrive.resetOdometry(Robot.toNoteTraj.getInitialPose()))
                // TODO: Tune PID vals via
                // //https://docs.jpsrobotics2554.org/programming/advanced-concepts/motion-profiling/
                // .andThen(Commands.runOnce(() -> mRotator.TODO(100))) // TODO: Test
                // Functionality, Change TalonFX Config Vals
                // .andThen(Commands.runOnce(() -> mLauncher.setLaunchVolts(12)))
                // .andThen(Commands.runOnce(() -> mLauncher.setFeedVolts(12)))
                .andThen(ramseteCommand)
                .andThen(Commands.runOnce(() -> mDrive.resetGyro()))
                // .andThen(toSpeakerRamsete)
                .andThen(new TurnToAngleProfiled(-180, mDrive).withTimeout(5))
                .andThen(Commands.runOnce(() -> mDrive.resetOdometry(Robot.toSpeakerTraj.getInitialPose())))
                .andThen(ramseteCommand2);
        // .andThen(Commands.runOnce(() -> mDrive.tankDriveVolts(0, 0)));
    }
}