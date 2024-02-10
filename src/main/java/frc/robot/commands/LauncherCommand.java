// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.common.types.input.ELogitech310;
import frc.robot.ADAM;

import frc.robot.Robot;
import frc.robot.RobotContainer;

import frc.robot.subsystems.LauncherSubsystem;
import lombok.Getter;




/**
 * An example command for use as a template.
 */

@SuppressWarnings("PMD.CommentSize")
public class LauncherCommand extends Command {


  private @Getter ADAM adam = new ADAM(null);

  private final @Getter LauncherSubsystem launcherSubsystem;

  /** Creates a new ExampleCommand. */
  public LauncherCommand(final LauncherSubsystem launcherSubsystem) {
    super();
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherSubsystem = launcherSubsystem;
    addRequirements(launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========== STARTING LAUNCHER COMMAND ==========");
    runTest(() -> {

    });
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    runTest(() -> {
      double launcherSpeed = 0;
      //input from buttons

      if(Robot.DATA.driverinput.isSet(ELogitech310.A_BTN))
      {
        launcherSpeed = 0.5;
      }

      // Set motor speeds in the IntakeSubsystem
      launcherSubsystem.setLaunchSpeed(launcherSpeed);
    });
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    runTest(() -> {

    });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Executes debugging actions for testing purposes.
   * Calls initialize(), execute(), and end() within a testing environment.
   *
   * @see #initialize()
   * @see #execute()
   * @see #end(boolean)
   */
  public void debugCommand() {
    runTest(() -> initialize());
    runTest(() -> execute());
    runTest(() -> end(false));
  }

  /**
   * Runs the provided code as a runnable task. If the code throws an exception,
   * it is caught, and an uncaught exception is passed to the default uncaught
   * exception handler for the current thread.
   *
   * @param code The runnable task to be executed.
   */
  public void runTest(final Runnable code) {
    try {
      code.run();
    } catch (Exception e) {
      adam.uncaughtException(Thread.currentThread(), e);
    }
  }
}
