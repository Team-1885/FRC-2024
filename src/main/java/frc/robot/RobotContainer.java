// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.commands.AmpLaunch;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.TalonFeed;
import frc.robot.commands.TalonRotate;
import frc.robot.commands.TalonShoot;
import frc.robot.commands.TalonShootSlow;
import frc.robot.subsystems.intake.TalonIntake;
import frc.robot.subsystems.drivetrain.CANDrivetrain;
import frc.robot.subsystems.launcher.CANLauncher;
import lombok.Getter;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("PMD.CommentSize")
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  @Getter private final CANDrivetrain mDrive;
  private final CANLauncher mLauncher;
  private final TalonIntake mIntake;

  private final TalonRotate mRotate;
  
  public final static CommandXboxController mDriverController = new CommandXboxController(1); // 1 is the USB Port to be used as indicated on the Driver Station
  public final static Joystick mOperatorController = new Joystick(2); // 2 is the USB Port to be used as indicated on the Driver Station
  CommandGenericHID controller = new CommandGenericHID(5);
  
  SendableChooser<Command> mChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        mDrive =
          new CANDrivetrain(new DriveIOSparkMaxBrushed());
          // we need to make a 

        mLauncher =
            new CANLauncher(
                new LauncherIOSparkMaxBrushed()); // Spark Max/Spark Flex + brushed, no encoders
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        mDrive = new CANDrivetrain(new DriveIOSim());
        mLauncher = new CANLauncher(new LauncherIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        mDrive = new CANDrivetrain(new CANDrivetrainIO() {});
        mLauncher = new CANLauncher(new CANLauncherIO() {});
        break;
    }

    // Set up note visualizer
    NoteVisualizer.setRobotPoseSupplier(drive::getPose);

    // Command Initialization
    mRotate = new TalonRotate(mIntake);

    mDrive.setDefaultCommand(
        mDrive.arcadeDriveCommand(
            () -> mDriverController.getLeftY(), () -> mDriverController.getRightX()));
    mIntake.setDefaultCommand(mRotate);

    mChooser.addOption("Curvy", loadTrajectory(Robot.trajectoryJSON, true));
    mChooser.addOption("Straight", loadTrajectory(Robot.trajectoryJSON, true));

    Shuffleboard.getTab("Autonomous").add(mChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  public Command loadTrajectory(String pFilename, boolean pResetOdometry) {
    RamseteCommand ramseteCommand = new RamseteCommand(Robot.trajectory, mDrive::getPose, new RamseteController(RobotMap.AutoConstants.kRamseteB, RobotMap.AutoConstants.kRamseteZeta), new SimpleMotorFeedforward(RobotMap.DriveConstants.ksVolts, RobotMap.DriveConstants.kvVoltSecondsPerMeter, RobotMap.DriveConstants.kaVoltSecondsSquaredPerMeter), RobotMap.DriveConstants.kDriveKinematics, mDrive::getWheelSpeeds, new PIDController(RobotMap.DriveConstants.kPDriveVel, 0, 0), new PIDController(RobotMap.DriveConstants.kPDriveVel, 0, 0), mDrive::tankDriveVolts, mDrive);
    if(pResetOdometry) {
        return new SequentialCommandGroup(new InstantCommand(()->mDrive.resetOdometry(Robot.trajectory.getInitialPose())), ramseteCommand);
    }
    else {
        return ramseteCommand;
    }
  }

  /**
   * Use this method to define your trigger->command mappings.
   * Triggers can be created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(mOperatorController,2)
        .whileTrue(
           new PrepareLaunch(mLauncher)
               .withTimeout(1)
               .andThen(new LaunchNote(mLauncher))
               .handleInterrupt(() -> mLauncher.stop()));
    
    new JoystickButton(mOperatorController, 3).whileTrue(mLauncher.feedlaunchWheel()).onFalse(new InstantCommand(mLauncher::stop));

    new JoystickButton(mOperatorController, 5)
        .whileTrue(
           new TalonFeed(mIntake)
               .handleInterrupt(() -> mIntake.stop()));

    new JoystickButton(mOperatorController, 6)
        .whileTrue(
           new TalonShoot(mIntake)
               .handleInterrupt(() -> mIntake.stop()));

    new JoystickButton(mOperatorController, 4)
        .whileTrue(
           new TalonShootSlow(mIntake)
               .handleInterrupt(() -> mIntake.stop()));
    /*new JoystickButton(mOperatorController, 7)
        .whileTrue(
           new PrepareLaunch(mLauncher)
               .withTimeout(1)
               .andThen(new AmpLaunch(mLauncher))
               .handleInterrupt(() -> mLauncher.stop()));*/

    
  }



/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { // the code needs to do stuff
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

 // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            Robot.trajectory,
            mDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            mDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            mDrive::tankDriveVolts,
            mDrive);

    // Reset odometry to the initial pose of the trajectory, run path following command, then stop at the end.
    return new WaitCommand(2)
        .andThen(Commands.runOnce(() -> mLauncher.setLaunchVolts(12)))
        .andThen(new WaitCommand(2))
        .andThen(Commands.runOnce(() -> mLauncher.setFeedVolts(12)))
        //.andThen(Commands.runOnce(() -> mLauncher.shootVolts(12, 12)))
        .andThen(new WaitCommand(2))
        .andThen(Commands.runOnce(() -> mLauncher.shootVolts(0, 0)));
  }

  public static CANDrivetrain getDrive() {
    return mDrive;
  }
}