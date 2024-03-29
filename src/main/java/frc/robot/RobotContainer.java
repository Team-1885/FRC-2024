// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AngleShooter;
import frc.robot.commands.AngleShooterUp;
import frc.robot.commands.ClimbLeft;
import frc.robot.commands.ClimbRight;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.MotionProfiling;
import frc.robot.commands.PositionControl;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.Reverse;
import frc.robot.commands.TalonFeed;
import frc.robot.commands.TalonRotate;
import frc.robot.commands.TalonShoot;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rotator;
import frc.robot.subsystems.DriveSubsystem;

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
    public final CANLauncher mLauncher = new CANLauncher();
    private final Rotator mRotator = new Rotator();
    private final Intake mIntake = new Intake();
    private final Climber mClimber = new Climber();
    private final ClimbRight mClimbRight;
    private final ClimbLeft mClimbLeft;
    private final Reverse mReverse;

    private final TalonRotate mRotate;
    private final AngleShooter mAngleShooter;
    private final AngleShooterUp mAngleShooterUp;

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
        mRotate = new TalonRotate(mRotator);
        mAngleShooter = new AngleShooter(mLauncher);
        mAngleShooterUp = new AngleShooterUp(mLauncher);
        mClimbRight = new ClimbRight(mClimber);
        mClimbLeft = new ClimbLeft(mClimber);
        mReverse = new Reverse(mClimber);

        mDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> mDrive.arcadeDrive(
                                -mDriverController.getLeftY(), -mDriverController.getRightX()),
                        mDrive));
        mRotator.setDefaultCommand(mRotate);
        //mLauncher.setDefaultCommand(mAngleShooter);
        mClimber.setDefaultCommand(mReverse);

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings.
     * Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        new JoystickButton(mOperatorController, 1)
                .whileTrue(
                        new PrepareLaunch(mLauncher)
                                .withTimeout(0.75)
                                .andThen(new LaunchNote(mLauncher))
                                .handleInterrupt(() -> mLauncher.stop()));

        new JoystickButton(mOperatorController, 2).whileTrue(mLauncher.feedlaunchWheel())
                .onFalse(new InstantCommand(mLauncher::stop));

        //new JoystickButton(mOperatorController, 4)
        //        .whileTrue(
        //                new MotionProfiling(mRotator)
        //                        .handleInterrupt(() -> mRotator.stop()));

        new JoystickButton(mOperatorController, 5)
                .whileTrue(
                        new TalonFeed(mIntake)
                                .handleInterrupt(() -> mIntake.stop()));

        new JoystickButton(mOperatorController, 6)
                .whileTrue(
                        new TalonShoot(mIntake)
                                .handleInterrupt(() -> mIntake.stop()));

        new JoystickButton(mOperatorController, 8)
                .whileTrue(new ClimbRight(mClimber).handleInterrupt(() -> mClimber.stop()));
        new JoystickButton(mOperatorController, 7)
                .whileTrue(new ClimbLeft(mClimber).handleInterrupt(() -> mClimber.stop()));

        //new JoystickButton(mOperatorController, 9)
        //        .whileTrue(new Reverse(mClimber).handleInterrupt(() -> mClimber.stop()));

        new JoystickButton(mOperatorController, 3).whileTrue(new AngleShooter(mLauncher).handleInterrupt(() -> mLauncher.stopServos()));
        new JoystickButton(mOperatorController, 9).whileTrue(new AngleShooterUp(mLauncher).handleInterrupt(() -> mLauncher.stopServos()));
        new JoystickButton(mOperatorController, 10).whileTrue(Commands.runOnce(() -> mLauncher.setServoPosition(60)));
        // DONT WORK //new JoystickButton(mOperatorController, 1).toggleOnTrue(Commands.startEnd(() -> mLauncher.setServoPosition(180), () -> mLauncher.setServoPosition(0), mLauncher));
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
                .andThen(Commands.runOnce(() -> mLauncher.setServoPosition(80)))
                .andThen(new WaitCommand(1))
                .andThen(new PrepareLaunch(mLauncher)).withTimeout(3)
                .andThen(new LaunchNote(mLauncher)).withTimeout(4)
                .andThen(Commands.runOnce(() -> mLauncher.stop()))
                .andThen(new WaitCommand(2));
                //.andThen(ramseteCommand);

                //.andThen(Commands.runOnce(() -> mLauncher.setLaunchVolts(12)))
                //.andThen(Commands.runOnce(() -> mLauncher.setFeedVolts(-12)))
    }
}