// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CANLauncher extends SubsystemBase {
  CANSparkMax mLaunchWheel;
  CANSparkMax mFeedWheel;
  Servo mServoL = new Servo(1);
  Servo mServoR = new Servo(2);


  /** Creates a new Launcher. */
  public CANLauncher() {
    mLaunchWheel = new CANSparkMax(Constants.LauncherConstants.kLauncherID, MotorType.kBrushless);
    mFeedWheel = new CANSparkMax(Constants.LauncherConstants.kFeederID, MotorType.kBrushless);

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
          setFeedWheel(Constants.LauncherConstants.kIntakeFeederSpeedSlower);
          setLaunchWheel(Constants.LauncherConstants.kIntakeLauncherSpeed);
        },
        // When the command stops, stop the wheels
        () -> {});
  }

    public Command feedWheel() {
    // The startEnd helper method takes a method to call when the command is initialized and one to call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(Constants.LauncherConstants.kIntakeFeederSpeed);
        },
        // When the command stops, stop the wheels
        () -> {});
  }

  public Command launchWheel() {
    // The startEnd helper method takes a method to call when the command is initialized and one to call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setLaunchWheel(Constants.LauncherConstants.kLaunchFeederSpeed);
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
          setLaunchWheel(-Constants.LauncherConstants.kFeedLaunchSpeed);
          setFeedWheel(Constants.LauncherConstants.kFeedLaunchSpeed);
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
    mLaunchWheel.set(Constants.LauncherConstants.kStopSpeed);
    mFeedWheel.set(Constants.LauncherConstants.kStopSpeed);
    mServoL.set(0);
    mServoR.set(0);
  }

  public void setServoPosition(double pos) {
    mServoL.setAngle(pos);
    mServoR.setAngle(pos);
  }

  public void stopServos() {
    mServoL.setDisabled();
    mServoR.setDisabled();
  }

    public Command setLeftPos(double pos) {
    // The startEnd helper method takes a method to call when the command is initialized and one to call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          mServoL.set(pos);
          mServoR.set(pos);
        },
        // When the command stops, stop the wheels
        () -> {
        });
  }

  public Command setRightPos(double pos) {
    // The startEnd helper method takes a method to call when the command is initialized and one to call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          mServoR.set(pos);
          mServoL.set(pos);
        },
        // When the command stops, stop the wheels
        () -> {
        });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LServo Pos", mServoL.getPosition());
    SmartDashboard.putNumber("RServo Pos", mServoR.getPosition());
    SmartDashboard.putNumber("LServo Speed", mServoL.getSpeed());
    SmartDashboard.putNumber("RServo Speed", mServoR.getSpeed());
    SmartDashboard.putNumber("LServo Angle", mServoL.getAngle());
    SmartDashboard.putNumber("RServo Angle", mServoR.getAngle());
  }
}