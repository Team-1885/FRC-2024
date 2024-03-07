// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CANLauncher extends SubsystemBase {
  CANSparkMax mLaunchWheel;
  CANSparkMax mFeedWheel;

  /** Creates a new Launcher. */
  public CANLauncher() {
    mLaunchWheel = new CANSparkMax(1, MotorType.kBrushless);
    mFeedWheel = new CANSparkMax(2, MotorType.kBrushless);

    mLaunchWheel.setSmartCurrentLimit(Constants.LauncherConstants.kLauncherCurrentLimit);
    mFeedWheel.setSmartCurrentLimit(Constants.LauncherConstants.kFeedCurrentLimit);
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside the subsytem is created to return an instance of a command. This works for commands that operate on only that subsystem, a similar approach can be done in RobotContainer for commands that need to span subsystems. The Subsystem class has helper methods, such as the startEnd method used here, to create these commands.
   */
  public Command getIntakeCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(-0.2);
          setLaunchWheel(-1);
        },
        // When the command stops, stop the wheels
        () -> {});
  }

    public Command feedWheel() {
    // The startEnd helper method takes a method to call when the command is initialized and one to call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(-1);
        },
        // When the command stops, stop the wheels
        () -> {});
  }

  public Command launchWheel() {
    // The startEnd helper method takes a method to call when the command is initialized and one to call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setLaunchWheel(-1);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  public Command feedlaunchWheel() {
    // The startEnd helper method takes a method to call when the command is initialized and one to call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setLaunchWheel(-0.5);
          setFeedWheel(-0.5);
        },
        // When the command stops, stop the wheels
        () -> {
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    mLaunchWheel.set(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    mFeedWheel.set(speed);
  }

  public void shootVolts(double pLaunchVolts, double pFeedVolts) {
    mLaunchWheel.setVoltage(pLaunchVolts);
    mFeedWheel.setVoltage(pFeedVolts);
  }

  public void setLaunchVolts(double pLaunchVolts) {
    mLaunchWheel.setVoltage(pLaunchVolts);
  }

  public void setFeedVolts(double pFeedVolts) {
    mFeedWheel.setVoltage(pFeedVolts);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    mLaunchWheel.set(0);
    mFeedWheel.set(0);
  }
}