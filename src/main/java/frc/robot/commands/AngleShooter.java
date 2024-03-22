// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CANLauncher;

public class AngleShooter extends Command {
  CANLauncher launcher;
  /** Creates a new Climb. */
  public AngleShooter(CANLauncher launcher) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcher = launcher;
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the wheels to launching speed
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.setServoPosition((RobotContainer.mOperatorController.getRawAxis(5) * 0.5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}