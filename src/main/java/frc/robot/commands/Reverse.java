// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Reverse extends Command {
  Climber mClimber;
  /** Creates a new Climb. */
  public Reverse(Climber mClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mClimber = mClimber;
    addRequirements(mClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the wheels to launching speed
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClimber.setClimberSpeed(RobotContainer.mOperatorController.getRawAxis(2), RobotContainer.mOperatorController.getRawAxis(3));

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