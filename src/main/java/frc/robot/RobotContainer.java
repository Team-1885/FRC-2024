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

        new JoystickButton(mOperatorController,7)
        .whileTrue(
           new PrepareLaunch(mLauncher)
               .withTimeout(1)
               .andThen(new LaunchNote(mLauncher))
               .handleInterrupt(() -> mLauncher.stop()));

        if (db.driverinput.isSet(InputMap.DRIVER.REFLECTIVE_TAPE_TRACKING)) {
            // adjust limelight pipeline so the robot targets reflective tape
            db.limelight.set(ELimelightData.TARGET_ID, Field2024.FieldElement.REFLECTIVE_TAPE);
            // let target lock take over
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
            // set LED
            //db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, 20);

            // log
            mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
            mTable.getEntry("Tracking Object").setString("Tracking Reflective Tape");

            //db.addressableled.set(EAddressableLEDData.DESIREDCOLOR, Enums.EAddressableLEDState.RED);
        }

        // if the cone tracking button is pressed
        else if (db.driverinput.isSet(InputMap.DRIVER.CONE_TRACKING)) {
            // adjust limelight pipeline so the robot targets cones
            db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CONE);
            // let target lock take over
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);

            // log
            mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
            mTable.getEntry("Tracking Object").setString("Tracking Cones");
            db.addressableled.set(EAddressableLEDData.DESIREDCOLOR, Enums.EAddressableLEDState.YELLOW);
        }

        // if the cube tracking button is pressed
        else if (db.driverinput.isSet(InputMap.DRIVER.CUBE_TRACKING)) {
            // adjust limelight pipeline so the robot targets cubes
            db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CUBE);
            // let target lock take over
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);

            // log
            mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
            mTable.getEntry("Tracking Object").setString("Tracking Cubes");
        }
        // track enemy robots' left corner
        else if (db.driverinput.isSet(InputMap.DRIVER.OPPONENT_ROBOT_LEFT_TRACKING)) {
            System.out.println("left tracking");
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
            {
                System.out.println("blue alliance");
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.RED_ROBOT_LEFT/*red left corner field element*/);
            }
            else // if our team is red, target blue robots
            {
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.BLUE_ROBOT_LEFT/*blue left corner field element*/);
            }
            // let target lock take over
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);

            // log
            mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
            mTable.getEntry("Tracking Object").setString("Tracking Cubes");
        }
        // track enemy robots' right
        else if (db.driverinput.isSet(InputMap.DRIVER.OPPONENT_ROBOT_RIGHT_TRACKING)) {
            // if our team is blue, target red robots
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
            {
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.RED_ROBOT_RIGHT/*red right corner field element*/);
            }
            else // if our team is red, target blue robots
            {
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.BLUE_ROBOT_RIGHT/*blue right corner field element*/);
            }
            // let target lock take over
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);

            // log
            mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
            mTable.getEntry("Tracking Object").setString("Tracking Cubes");
        }
        else { // No targetting button is pressed
            // set limelight pipeline back to base camera without crazy filters
            db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CAMERA.id());
            // give turning control back to driver
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.VELOCITY);

            // log
            mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
            mTable.getEntry("Tracking Object").setString("Not Tracking");
            db.addressableled.set(EAddressableLEDData.DESIREDCOLOR, Enums.EAddressableLEDState.BATTLEFIElD_COLOR);
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
