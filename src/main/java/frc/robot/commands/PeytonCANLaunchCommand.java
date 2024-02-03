// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PeytonCANLauncher;
import lombok.Getter;


public class PeytonCANLaunchCommand extends CommandBase {

private final @Getter PeytonCANLauncher peytonCANLauncher;

  // Creates a new PeytonCANLaunchCommand.
  public PeytonCANLaunchCommand(final PeytonCANLauncher peytonCANLauncher) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.peytonCANLauncher = peytonCANLauncher;
    addRequirements(peytonCANLauncher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    peytonCANLauncher.setLaunchWheel(1.0); //value found in kit bot constants
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Do I still need to keep the method if there is nothing in it?
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    peytonCANLauncher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}