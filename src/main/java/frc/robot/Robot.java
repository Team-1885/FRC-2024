// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Path;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in the project.
 */

@SuppressWarnings("PMD.CommentSize")
public class Robot extends TimedRobot {
	private Command mAutonomousCommand;

	RobotContainer mRobotContainer;

	private DriveSubsystem mWestCoastDrive = DriveSubsystem.getInstance();

	public static String trajJSON =
		"Paths/output/MoveToNote.wpilib.json";
	public static String traj2JSON =
		"Paths/output/ReturnToSpeaker.wpilib.json";
	public static Trajectory toNoteTraj = new Trajectory();
	public static Trajectory toSpeakerTraj = new Trajectory();

	/**
	 * Default constructor for the Robot class. This constructor is automatically invoked when an instance of the Robot class is created.
	 * 
	 * Initializes the Robot instance by calling the no-argument constructor of the superclass (TimedRobot).
	 */
	public Robot() {
		super();
	}

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit() {
		mRobotContainer = new RobotContainer();

		LiveWindow.disableAllTelemetry();

		mWestCoastDrive.zeroHeading();
		mWestCoastDrive.resetEncoders();
		//mWestCoastDrive.resetGyro();

		loadMoveToNote();
		loadMoveToSpeaker();

		// Make sure you only configure port forwarding once in your robot code.
        // Do not place these function calls in any periodic functions
        for (int port = 5800; port <= 5807; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }
	}
 
	public void loadMoveToNote() {
		try {
			Path trajectoryPath =
				Filesystem.getDeployDirectory().toPath().resolve(
					trajJSON);
			toNoteTraj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError(
				"Unable to open trajectory: " + trajJSON,
				ex.getStackTrace());
		}
	}

	public void loadMoveToSpeaker() {
		try {
			Path trajectory2Path =
				Filesystem.getDeployDirectory().toPath().resolve(
					traj2JSON);
			toSpeakerTraj = TrajectoryUtil.fromPathweaverJson(trajectory2Path);
		} catch (IOException ex) {
			DriverStation.reportError(
				"Unable to open trajectory: " + traj2JSON,
				ex.getStackTrace());
		}
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran during disabled, autonomous, teleoperated and test.
	 * 
	 * This runs after the mode specific periodic functions, but beforecLiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled commands, running already-scheduled commands, removing finished or interrupted commands, and running subsystem periodic() methods. This must be called from the robot's periodic block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		mAutonomousCommand = mRobotContainer.getAutonomousCommand();

		// schedule the autonomous command
		if (mAutonomousCommand != null) {
			mAutonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to continue until interrupted by another command, remove this line or comment it out.
		if (mAutonomousCommand != null) {
			mAutonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {}
}
