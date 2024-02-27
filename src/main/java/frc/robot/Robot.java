// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.hardware.vendors.firstparties.Clock;
import frc.robot.hardware.vendors.firstparties.Settings;

import java.io.IOException;
import java.nio.file.Path;

import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.ICodexTimeProvider;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 * If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */

@SuppressWarnings("PMD.CommentSize")
public class Robot extends TimedRobot {
	RobotContainer mRobotContainer;
	private Command mAutonomousCommand;
	private final SysIdRoutineBot mRobot = new SysIdRoutineBot();

	public static final Clock CLOCK =
		(RobotBase.isReal() ? new Clock() : new Clock().simulated());
	public static final Field2d FIELD = new Field2d();
	public static final boolean IS_SIMULATED = RobotBase.isSimulation();
	public static String CLIMB_MODE = "";

	public static String trajectoryJSON =
		"Paths/output/PathWeaver_Straight.wpilib.json";
	public static Trajectory trajectory = new Trajectory();

	/**
	 * Default constructor for the Robot class. This constructor is
	 * automatically invoked when an instance of the Robot class is created.
	 * Initializes the Robot instance by calling the no-argument constructor of
	 * the superclass (TimedRobot).
	 */
	public Robot() {
		super();
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Starts recording to data log
		DataLogManager.start();

		// Record both DS control and joystick data
		DriverStation.startDataLog(DataLogManager.getLog());

		// (alternatively) Record only DS control data
		DriverStation.startDataLog(DataLogManager.getLog(), false);

		mRobot.configureBindings();

		CLOCK.update();
		DataLogManager.log("===> ROBOT INIT Starting");

		// Measuring Initialization Duration
		Timer initTimer = new Timer();
		initTimer.start();

		DataLogManager.log("Starting Robot Initialization...");
		ICodexTimeProvider provider = new ICodexTimeProvider() {
			@Override
			public double getTimestamp() {
				return CLOCK.now();
			}
		};
		CodexMetadata.overrideTimeProvider(provider);

		LiveWindow.disableAllTelemetry();

		/*
		 * Some things need to wait until after the robot connects to the DS. So
		 * keep this thread here.
		 */
		new Thread(new DSConnectInitThread()).start();

		initTimer.stop();
		DataLogManager.log("Robot initialization finished in " + initTimer.get() + " seconds");

		if (!Settings.kIsLogging) {
			DataLogManager.log("------------Not Logging to CSV------------");
		}
		sigma();
		mRobotContainer = new RobotContainer();
	}

	public void sigma() {
		try {
			Path trajectoryPath =
				Filesystem.getDeployDirectory().toPath().resolve(
					trajectoryJSON);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError(
				"Unable to open trajectory: " + trajectoryJSON,
				ex.getStackTrace());
		}
	}

	/**
	 * These things rely on match metadata, so we need to wait for the DS to
	 * connect
	 */
	private void initAfterConnection() {}

	/**
	 * This function is called every 20 ms, no matter the mode.
	 * Use this for items like diagnostics that you want ran during disabled,
	 * autonomous, teleoperated and test.
	 * This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.
		// This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands, and running subsystem periodic() methods. This
		// must be called from the robot's periodic block in order for anything
		// in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		SmartDashboard.putData(FIELD);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
		DataLogManager.log("Disabled Initialization");
	}

	@Override
	public void disabledPeriodic() {}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		mAutonomousCommand = mRobotContainer.getAutonomousCommand();
		if (mAutonomousCommand != null) {
		  mAutonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts
		// running. If you want the autonomous to continue until interrupted by
		// another command, remove this line or comment it out.
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
	public void simulationInit() {
		// ...
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
		// ...
	}

	public String toString() {
		String mRobotMode = "Unknown";
		String mRobotEnabledDisabled = "Unknown";
		double mNow = Timer.getFPGATimestamp();

		if (this.isAutonomous()) {
			mRobotMode = "Autonomous";
		}
		if (this.isTest()) {
			mRobotEnabledDisabled = "Test";
		}
		if (this.isEnabled()) {
			mRobotEnabledDisabled = "Enabled";
		}
		if (this.isDisabled()) {
			mRobotEnabledDisabled = "Disabled";
		}

		return String.format("State: %s\tMode: %s\tTime: %s",
			mRobotEnabledDisabled, mRobotMode, mNow);
	}

	private class DSConnectInitThread implements Runnable {
		@Override
		public void run() {
			while (!DriverStation.isDSAttached()) {
				try {
					DataLogManager.log("Waiting on Robot <--> DS Connection...");
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			initAfterConnection();
		}
	}
}
