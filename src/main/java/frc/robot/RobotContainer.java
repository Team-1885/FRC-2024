// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.WestCoastDrive;
import lombok.Getter;

/**
 * Responsible for initializing and configuring the robot's subsystems,
 * commands, and trigger mappings.
 * 
 * This is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
@SuppressWarnings("PMD.CommentSize")
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private @Getter final WestCoastDrive mWestCoastDrive;
  private @Getter final DriveCommand mDriveCommand;
  private @Getter final ExampleSubsystem mExampleSubsystem;
  private @Getter final ExampleCommand mExampleCommand;
  private @Getter final XboxController xboxController = new XboxController(RobotMap.DriverConstants.D_XBOX_PORT);
  public @Getter final static Joystick logitech = new Joystick(RobotMap.DriverConstants.D_LOGITECH_PORT);
  private final SendableChooser<Command> autoChooser;
  private final Field2d mField;

  /**
   * The container for the robot. 
   * Responsible for initializing and configuring the robot's
   * subsystems, commands, OI, and trigger mappings.
   */
  public RobotContainer() {
    // Subsystem Initialization
    mWestCoastDrive = WestCoastDrive.getInstance();
    mExampleSubsystem = new ExampleSubsystem();

    // Register Named Commands
    NamedCommands.registerCommand("DriveCommand", new ExampleCommand(mExampleSubsystem));
    NamedCommands.registerCommand("DriveCommand", new DriveCommand(mWestCoastDrive));

    // Command Initialization
    mDriveCommand = new DriveCommand(mWestCoastDrive);
    mExampleCommand = new ExampleCommand(mExampleSubsystem);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    SmartDashboard.putData("Auto Chooser", autoChooser);
    mWestCoastDrive.setDefaultCommand(mDriveCommand);

    mField = new Field2d();
    SmartDashboard.putData("Field", mField);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      mField.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      mField.getObject("Target Pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      mField.getObject("Path").setPoses(poses);
    });

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
    // ...
  }

  /**
   * Returns a command that follows a predefined path.
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run during the autonomous period.
   */
  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath mPath = PathPlannerPath.fromPathFile("Drive Straight");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(mPath);
    // return autoChooser.getSelected();
  }
}
