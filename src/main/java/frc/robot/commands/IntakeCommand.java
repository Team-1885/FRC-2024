// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.common.types.input.ELogitech310;
import frc.robot.ADAM;

import frc.robot.Robot;
import frc.robot.RobotContainer;

import frc.robot.subsystems.IntakeSubsystem;
import lombok.Getter;




/**
 * An example command for use as a template.
 */

@SuppressWarnings("PMD.CommentSize")
public class IntakeCommand extends Command {


  private @Getter ADAM adam = new ADAM(null);

  private final @Getter IntakeSubsystem intakeSubsystem;

  /** Creates a new ExampleCommand. */
  public IntakeCommand(final IntakeSubsystem intakeSubsystem) {
    super();
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========== STARTING INTAKE COMMAND ==========");
    runTest(() -> {

    });
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    runTest(() -> {
      double feedSpeed = 0;
      double rotateSpeed = 0;
      //input from buttons
      if(Robot.DATA.driverinput.isSet(ELogitech310.A_BTN))
      {
        feedSpeed = 0.5;
      }
      
      /*//rotate up
      if(Robot.DATA.driverinput.isSet(ELogitech310.LEFT_TRIGGER_AXIS))
      {
        rotateSpeed = 0.5;
      }
      //rotate down
      if(Robot.DATA.driverinput.isSet(ELogitech310.L_BTN))
      {
        rotateSpeed = -0.5;
      }*/

      rotateSpeed = RobotContainer.logitech2.getRawAxis(1) * 1;
      //double turnSpeed = RobotContainer.logitech.getZ() * 0.7; // Get X-axis value of left stick //AVI DID THIS

      // You may want to add deadzones to prevent small joystick values from causing
      // unintended movement

      rotateSpeed = applyDeadzone(rotateSpeed, 0.05);
      //turnSpeed = applyDeadzone(turnSpeed, 0.05);

      // Set motor speeds in the IntakeSubsystem
      intakeSubsystem.setFeederSpeed(feedSpeed);
      intakeSubsystem.setRotaterSpeed(rotateSpeed);
    });
  }

    // Helper method to apply a deadzone to joystick values
    private double applyDeadzone(double value, double deadzone) {
      if (Math.abs(value) < deadzone) {
        return 0.0;
      } else {
        return value;
      }
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
