// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ADAM;
import frc.robot.subsystems.PneumaticsTest;
import lombok.Getter;

public class PneumaticsCommand extends CommandBase {
  /** Creates a new PneumaticsCommand. */
  private @Getter ADAM adam = new ADAM(null);
  private @Getter PneumaticsTest pneumaticsTest;

  public PneumaticsCommand(final PneumaticsTest pneumaticsTest) {
    // Use addRequirements() here to declare subsystem dependencies.
    super();
    // Use addRequirements() here to declare subsystem dependencies.
    this.pneumaticsTest = pneumaticsTest;
    addRequirements(pneumaticsTest);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runTest(() -> {
    });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runTest(() -> {

    });
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runTest(() -> {

    });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
  public void debugCommand() {
    runTest(() -> initialize());
    runTest(() -> execute());
    runTest(() -> end(false));
  }

  public void runTest(final Runnable code) {
    try {
      code.run();
    } catch (Exception e) {
      adam.uncaughtException(Thread.currentThread(), e);
    }
  }
}