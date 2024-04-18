// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  Climber climber;

  /** Creates a new Climb. */
  public Climb(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean lPressed = RobotContainer.mOperatorController.getRawButton(ClimberConstants.climberLButtonID);
    boolean rPressed = RobotContainer.mOperatorController.getRawButton(ClimberConstants.climberRButtonID);

    double l = lPressed ? 1 : 0;
    double r = rPressed ? 1 : 0;

    climber.setClimberSpeed(RobotContainer.mOperatorController.getRawAxis(2) - l, RobotContainer.mOperatorController.getRawAxis(3) - r);
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

