// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
//import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PeytonLaunchCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.WestCoastDrive;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PeytonLauncher;
import lombok.Getter;

/** 
 * This class is where the bulk of the robot should be declared. 
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls). 
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("PMD.CommentSize") public class RobotContainer {

  
// The robot's subsystems and commands are defined here...
  private @Getter final WestCoastDrive westCoastDrive = WestCoastDrive.getInstance();
  private @Getter final DriveCommand driveCommand = new DriveCommand(westCoastDrive);
  private @Getter final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  private @Getter final ExampleCommand exampleCommand = new ExampleCommand(exampleSubsystem);
  private @Getter final PeytonLauncher peytonCANLauncher  = new PeytonLauncher();
  private @Getter final PeytonLaunchCommand peytonCANLaunchCommand = new PeytonLaunchCommand(peytonCANLauncher);
  

  //private @Getter final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  //private @Getter final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  private @Getter final XboxController xboxController = new XboxController(RobotMap.DriverConstants.D_XBOX_PORT);
  public @Getter final static Joystick logitech1 = new Joystick(RobotMap.DriverConstants.D_LOGITECH_PORT);
  
  private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    SmartDashboard.putData("Auto Chooser", autoChooser);
    westCoastDrive.setDefaultCommand(driveCommand);
    NamedCommands.registerCommand("DriveCommand", driveCommand);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. 
   * Triggers can be created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Add code here

    // Trigger driveTriggerX = new Trigger(() -> logitech.getRawAxis(0) > 0.01); // Replace 0 with the axis number for the X axis
    // driveTriggerX.whileTrue(driveCommand);

    // Trigger driveTriggerY = new Trigger(() -> logitech.getRawAxis(1) > 0.01); // Replace 1 with the axis number for the Y axis
    // driveTriggerY.whileTrue(driveCommand);
   
    logitech1.getRawAxis(0); // X
    logitech1.getRawAxis(1); // Y
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("New Path");
    PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");
    
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }
  public Command getAutoChooserCommand() {
    return autoChooser.getSelected();
  }
}
