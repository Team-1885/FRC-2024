// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ADAM;
import frc.robot.subsystems.TalonIntake;
import lombok.Getter;

public class TalonCommand extends Command {

private @Getter ADAM adam = new ADAM(null);

private final @Getter TalonIntake talonIntake;

  /** Creates a new TalonCommand. */
  public TalonCommand(final TalonIntake talonIntake) {
    super();
    // Use addRequirements() here to declare subsystem dependencies.
    this.talonIntake = talonIntake;
    addRequirements(talonIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========== STARTING TALON COMMAND ==========");
    runTest(() -> {

    });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {runTest(() -> {
     
    });}

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

  //add button code somewhere in here or in the subsystem

  public void runTest(final Runnable code) {
    try {
      code.run();
    } catch (Exception e) {
      adam.uncaughtException(Thread.currentThread(), e);
    }
  }
}
