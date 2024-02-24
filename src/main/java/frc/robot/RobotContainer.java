// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.WC;
import lombok.Getter;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("PMD.CommentSize")
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private @Getter final WC mWestCoastDrive = WC.getInstance();
  private @Getter final DriveCommand mDriveCommand;
  private @Getter final CANLauncher mLauncher = new CANLauncher();
  public @Getter final static Joystick mDriverController = new Joystick(1); // 1 is the USB Port to be used as indicated on the Driver Station
  CommandGenericHID controller = new CommandGenericHID(5);
  private final Field2d mField;
  SendableChooser<Command> mChooser = new SendableChooser<>();
  public @Getter final static Joystick mOperatorController = new Joystick(2); // 2 is the USB Port to be used as indicated on the Driver Station

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Register Named Commands
    NamedCommands.registerCommand("DriveCommand", new DriveCommand(mWestCoastDrive));

    // Command Initialization
    mDriveCommand = new DriveCommand(mWestCoastDrive);


    // Another option that allows you to specify the default auto by its name
    mWestCoastDrive.setDefaultCommand(mDriveCommand);

    mField = new Field2d();

    mChooser.addOption("Curvy", loadTrajectory(Robot.trajectoryJSON, true));
    mChooser.addOption("Straight", loadTrajectory(Robot.trajectoryJSON, true));

    Shuffleboard.getTab("Autonomous").add(mChooser);

    SmartDashboard.putData("Field", mField);

    // Configure the trigger bindings
    configureBindings();
  }

  public Command loadTrajectory(String pFilename, boolean pResetOdometry) {
    RamseteCommand ramseteCommand = new RamseteCommand(Robot.trajectory, mWestCoastDrive::getPose, new RamseteController(RobotMap.AutoConstants.kRamseteB, RobotMap.AutoConstants.kRamseteZeta), new SimpleMotorFeedforward(RobotMap.DriveConstants.ksVolts, RobotMap.DriveConstants.kvVoltSecondsPerMeter, RobotMap.DriveConstants.kaVoltSecondsSquaredPerMeter), RobotMap.DriveConstants.kDriveKinematics, mWestCoastDrive::getWheelSpeeds, new PIDController(RobotMap.DriveConstants.kPDriveVel, 0, 0), new PIDController(RobotMap.DriveConstants.kPDriveVel, 0, 0), mWestCoastDrive::tankDriveVolts, mWestCoastDrive);
    if(pResetOdometry) {
        return new SequentialCommandGroup(new InstantCommand(()->mWestCoastDrive.resetOdometry(Robot.trajectory.getInitialPose())), ramseteCommand);
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
    new JoystickButton(mOperatorController, 1)
        .whileTrue(
           new PrepareLaunch(mLauncher)
                .withTimeout(1)
               .andThen(new LaunchNote(mLauncher))
               .handleInterrupt(() -> mLauncher.stop()));
    
    new JoystickButton(mOperatorController, 2).whileTrue(mLauncher.feedlaunchWheel()).onFalse(new InstantCommand(mLauncher::stop));
  }

/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
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
            mWestCoastDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            mWestCoastDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            mWestCoastDrive::tankDriveVolts,
            mWestCoastDrive);

    // Reset odometry to the initial pose of the trajectory, run path following command, then stop at the end.
    return Commands.runOnce(() -> mWestCoastDrive.resetOdometry(Robot.trajectory.getInitialPose()))
        .andThen(ramseteCommand)
        .andThen(Commands.runOnce(() -> mWestCoastDrive.tankDriveVolts(0, 0)));
  }
}

