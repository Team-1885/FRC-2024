// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.ShooterConstants.LauncherConstants.*;


public class CANLauncher extends SubsystemBase {
  CANSparkMax mLaunchWheel;
  CANSparkMax mFeedWheel;

  /** Creates a new Launcher. */
  public CANLauncher() {
    System.out.println("CANLauncher is Constructed");
    mLaunchWheel = new CANSparkMax(1, MotorType.kBrushed);
    mFeedWheel = new CANSparkMax(3, MotorType.kBrushed);
    System.out.println("mLaunchWheel is Constructed");
    System.out.println("mFeedWheel is Constructed");

    // mLaunchWheel.setSmartCurrentLimit(kLauncherCurrentLimit);
    // mFeedWheel.setSmartCurrentLimit(kFeedCurrentLimit);
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside the subsytem is created to return an instance of a command. This works for commands that operate on only that subsystem, a similar approach can be done in RobotContainer for commands that need to span subsystems. The Subsystem class has helper methods, such as the startEnd method used here, to create these commands.
   */
  public Command getIntakeCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(-1);
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
          setLaunchWheel(-1 * 5);
          setFeedWheel(-1);
        },
        // When the command stops, stop the wheels
        () -> {
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    System.out.println("setLaunchWheel() is Called");
    mLaunchWheel.set(speed);
    System.out.println("mLaunchWheel set to speed " + speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    System.out.println("setFeedWheel() is Called");
    mFeedWheel.set(speed);
    System.out.println("mFeedWheel set to speed " + speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    System.out.println("stop() is Called");
    mLaunchWheel.set(0);
    mFeedWheel.set(0);
    System.out.println("mLaunchWheel & mFeedWheel set to speed 0.0");
  }
}
