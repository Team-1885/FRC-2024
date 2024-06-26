// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Rotator;


/*
 * This is an example of creating a command as a class. The base Command class provides a set of methods that your command will override.
 */
public class TalonRotate extends Command {
  Rotator mRotater;

  /** Creates a new LaunchNote. */
  public TalonRotate(Rotator pRotater) {
    // save the launcher system internally
    mRotater = pRotater;

    // indicate that this command requires the launcher system
    addRequirements(mRotater);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = -RobotContainer.mOperatorController.getRawAxis(1) * 0.2;

    double pos = RobotContainer.mRotator.getPos();

    double max = 39.15576171875;
    double min = -3.2744140625;
    double space = 5.0;

    if (speed > 0) {
      if (pos >= max)
        speed = 0;
      else if (pos >= max - space)
        speed *= 0.5;
    }

    if (speed < 0) {
      if (pos <= min)
        speed = 0;
      else if (pos <= min + space)
        speed *= 0.1;
    }

    mRotater.setIntakeRotater(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the scheduler to end the command when the button is released.
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.
    mRotater.stop();
  }
}