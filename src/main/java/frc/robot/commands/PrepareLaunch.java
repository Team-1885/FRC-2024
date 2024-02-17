// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.LauncherConstants.*;
import frc.robot.subsystems.CANLauncher;

public class PrepareLaunch extends Command {
  CANLauncher mLauncher;

  /** Creates a new PrepareLaunch. */
  public PrepareLaunch(CANLauncher pLauncher) {
    // save the launcher system internally
    mLauncher = pLauncher;
    System.out.println("PrepareLaunch object is made");

    // indicate that this command requires the launcher system
    addRequirements(mLauncher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Prepare Launch Initialized");
    // Set launch wheel to speed, keep feed wheel at 0 to let launch wheel spin up.
    mLauncher.setLaunchWheel(kLauncherSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Prepare launch is executed");
    // There is nothing we need this command to do on each iteration. You could remove this method and the default blank method of the base class will run.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Prepare launch is ended");
    // Do nothing when the command ends. The launch wheel needs to keep spinning in order to launch
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout decorator on the command to end it.
    return false;
  }
}