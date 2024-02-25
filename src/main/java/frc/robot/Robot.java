// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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

import lombok.Getter;

import java.util.Arrays;

import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.ICodexTimeProvider;
import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ELevel;
import com.flybotix.hfr.util.log.ILog;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 * If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */

@SuppressWarnings("PMD.CommentSize")
public class Robot extends TimedRobot {
	private Command mAutonomousCommand;
	private final SysIdRoutineBot mRobot = new SysIdRoutineBot();

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
    // Configure default commands and condition bindings on robot startup
	DataLogManager.start();
	DriverStation.startDataLog(DataLogManager.getLog());
	DriverStation.startDataLog(DataLogManager.getLog(), false);
    mRobot.configureBindings();
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
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

	  /** This function is called once each time the robot enters Disabled mode. */
	  @Override
	  public void disabledInit() {}
	
	  @Override
	  public void disabledPeriodic() {}
	
	  @Override
	  public void autonomousInit() {
		mAutonomousCommand = mRobot.getAutonomousCommand();
	
		if (mAutonomousCommand != null) {
		  mAutonomousCommand.schedule();
		}
	  }
	
	  /** This function is called periodically during autonomous. */
	  @Override
	  public void autonomousPeriodic() {}
	
	
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
}
