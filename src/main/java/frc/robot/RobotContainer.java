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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.TalonFeed;
import frc.robot.commands.TalonRotate;
import frc.robot.commands.TalonShoot;
import frc.robot.commands.TalonShootSlow;
import frc.robot.commands.TurnToAngle;
import frc.robot.hardware.vendors.thirdparties.LL3.ELimelightData;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rotator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.limelight.LimelightNT;
import frc.robot.LimelightHelpers;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("PMD.CommentSize")
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static final DriveSubsystem mDrive = DriveSubsystem.getInstance();
  private final CANLauncher mLauncher = new CANLauncher();
  private final Rotator mRotator = new Rotator();
  private final Intake mIntake = new Intake();


  private final TalonRotate mRotate;
  private final double kP = 0.012;

  public final static CommandXboxController mDriverController = new CommandXboxController(1); // 1 is the USB Port to be used as indicated on the Driver Station
  public final static Joystick mOperatorController = new Joystick(2); // 2 is the USB Port to be used as indicated on the Driver Station

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Command Initialization
    mRotate = new TalonRotate(mRotator);

    mDrive.setDefaultCommand(
        mDrive.arcadeDriveCommand(
            () -> -mDriverController.getLeftY(), () -> -mDriverController.getRightX()));
    mRotator.setDefaultCommand(mRotate);

    // Configure the trigger bindings
    configureBindings();
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

    
    if (Robot.mode() == EMatchMode.TELEOPERATED) {
        /**
        * Upon Specific Buttons Pressed, adjust the
        * limelight pipeline to target different objects
        *
        * NOTE: More tracking stuff in BaseManualController line 43
        *
        * To add a new pipeline, you need to add a Field Element with the dimensions and pipeline id
        * in the Field2022 class. You also need to add the button used for the new element
        * to an if statement in BaseManual Controller.
        *
        * We get the pipeline we want to track from the field element we want to track
        * Not from a random constant in Settings.java
        *
        * Make sure normal vision pipeline doesn't target anything to avoid weird bugs
        */
        // if the reflective tape button is pressed
        /*if (db.driverinput.isSet(ELogitech310.RIGHT_TRIGGER_AXIS)) {
            System.out.println("right trigger");
        }*/

        new JoystickButton(mOperatorController,9)
        .whileTrue(
           new PrepareLaunch(mLauncher)
               .withTimeout(1)
               .andThen(new LaunchNote(mLauncher))
               .handleInterrupt(() -> mLauncher.stop()));

    
        float Kp = -0.1f;
        float min_command = 0.05f;
        
        //std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
        //float tx = table->GetNumber("tx");

        

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        //FloatArraySubscriber tx;

        //FloatTopic tx = table.getFloatTopic("tx");
        //float tx = table->GetNumber("tx");

        FloatArraySubscriber val = table.getFloatArrayTopic("limelight").subscribe(new float[] {});

        float[] txArr = val.get();

        float tx = txArr[txArr.length-1];

        if (joystick->GetRawButton(9))
        {
            float heading_error = -tx;
            float steering_adjust = 0.0f;
            if (Math.abs(heading_error) > 1.0) 
            {
                if (heading_error < 0) 
                {
                    steering_adjust = Kp*heading_error + min_command;
                } 
                else 
                {
                    steering_adjust = Kp*heading_error - min_command;
                }
            } 
            left_command += steering_adjust;
            right_command -= steering_adjust;
        }      

    }
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
                Constants.DrivetrainConstants.ksVolts,
                Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
                Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DrivetrainConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.DrivetrainConstants.kMaxSpeedMetersPerSecond,
                Constants.DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DrivetrainConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    RamseteCommand toNoteRamsete =
        new RamseteCommand(
            Robot.toNoteTraj,
            mDrive::getPose,
            new RamseteController(Constants.DrivetrainConstants.kRamseteB, Constants.DrivetrainConstants.kRamseteZeta),
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

    RamseteCommand toSpeakerRamsete =
        new RamseteCommand(
            Robot.toSpeakerTraj,
            mDrive::getPose,
            new RamseteController(Constants.DrivetrainConstants.kRamseteB, Constants.DrivetrainConstants.kRamseteZeta),
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

    // Reset odometry to the initial pose of the trajectory, run path following command, then stop at the end.
    return Commands.runOnce(() -> mDrive.resetOdometry(Robot.toNoteTraj.getInitialPose()))
        .andThen(new TurnToAngle(90, mDrive))
        .andThen(Commands.runOnce(() -> mRotator.TODO(100))) // TODO: Test Functionality, Change TalonFX Config Vals
        //.andThen(Commands.runOnce(() -> mLauncher.setLaunchVolts(12)))
        //.andThen(Commands.runOnce(() -> mLauncher.setFeedVolts(12)))
        //.andThen(toNoteRamsete)
        //.andThen(toSpeakerRamsete)
        .andThen(Commands.runOnce(() -> mDrive.tankDriveVolts(0, 0)));
  }
}
