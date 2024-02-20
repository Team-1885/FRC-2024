// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.WestCoastDrive;
import lombok.Getter;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.LaunchNote;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("PMD.CommentSize")
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private @Getter final WestCoastDrive mWestCoastDrive = WestCoastDrive.getInstance();
  private @Getter final DriveCommand mDriveCommand;
  CommandGenericHID controller = new CommandGenericHID(5);
  private @Getter final CANLauncher mLauncher = new CANLauncher();
  public @Getter final static Joystick logitech = new Joystick(0);
  private final Field2d mField;
  SendableChooser<Command> mChooser = new SendableChooser<>();

  public @Getter final static Joystick mOperatorController = new Joystick(1);

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

    mChooser.addOption("Curvy", loadTrajectory("Paths/PathWeaver_Curvy.path", true));
    mChooser.addOption("Straight", loadTrajectory("Paths/PathWeaver_Straight.path", true));

    Shuffleboard.getTab("Autonomous").add(mChooser);

    SmartDashboard.putData("Field", mField);

    // Configure the trigger bindings
    configureBindings();
  }

  public Command loadTrajectory(String pFilename, boolean pResetOdometry) {
    Trajectory trajectory;
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pFilename);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch (IOException pException){
        DriverStation.reportError("Unable to open trajectory" + pFilename, pException.getStackTrace());
        System.out.println("Unable to read from file " + pFilename);
        return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, mWestCoastDrive::getPose, new RamseteController(RobotMap.AutoConstants.kRamseteB, RobotMap.AutoConstants.kRamseteZeta), new SimpleMotorFeedforward(RobotMap.DriveConstants.ksVolts, RobotMap.DriveConstants.kvVoltSecondsPerMeter, RobotMap.DriveConstants.kaVoltSecondsSquaredPerMeter), RobotMap.DriveConstants.kDriveKinematics, mWestCoastDrive::getWheelSpeeds, new PIDController(RobotMap.DriveConstants.kPDriveVel, 0, 0), new PIDController(RobotMap.DriveConstants.kPDriveVel, 0, 0), mWestCoastDrive::tankDriveVolts, mWestCoastDrive);
    if(pResetOdometry) {
        return new SequentialCommandGroup(new InstantCommand(()->mWestCoastDrive.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
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
    controller.button(0).onTrue(mDriveCommand);
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
    return mChooser.getSelected();
  }
}

