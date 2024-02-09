// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.common.types.input.ELogitech310;
import frc.robot.Robot;

public class LaunchNoteCommand extends Command {
  /** Creates a new LaunchNoteSubsystem. */
  public LaunchNoteCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runTest(() -> {
      double launchSpeed = 0;
      
      //input from buttons
      if(Robot.DATA.driverinput.isSet(ELogitech310.A_BTN))
      {
        launchSpeed = 0.5;
      }
      

      // Set motor speeds in the IntakeSubsystem
      LaunchNoteCommand.setLaunchMotorSpeed(launchSpeed);
      
    });
  

  // Called once the command ends or is interrupted.
  }

  private static void setLaunchMotorSpeed(double launchSpeed) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setLaunchMotorSpeed'");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
   public void runTest(final Runnable code) {
    try {
      code.run();
    } catch (Exception e) {
      
    }
}
}