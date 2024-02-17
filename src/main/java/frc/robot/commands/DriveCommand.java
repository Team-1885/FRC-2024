// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ADAM;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WestCoastDrive;
import lombok.Getter;

/**
 * An example command for use as a template.
 */

@SuppressWarnings("PMD.CommentSize")
public class DriveCommand extends Command {

  /**
   * Lorem Ipsum.
   */
  private @Getter ADAM adam = new ADAM(null);

  /**
   * Lorem Ipsum.
   */
  private final @Getter WestCoastDrive westCoastDrive;

  /** Creates a new ExampleCommand. */
  public DriveCommand(final WestCoastDrive westCoastDrive) {
    super();
    // Use addRequirements() here to declare subsystem dependencies.
    this.westCoastDrive = westCoastDrive;
    addRequirements(westCoastDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========== STARTING DRIVECOMMAND ==========");
    runTest(() -> {

    });
  }

  // Fixed by Avi Sharma
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runTest(() -> {
      
      double forwardSpeed = RobotContainer.logitech.getRawAxis(1) * 0.4;
      double turnSpeed = RobotContainer.logitech.getRawAxis(4) * 0.4; // Get X-axis value of left stick

      // You may want to add deadzones to prevent small joystick values from causing
      // unintended movement
      forwardSpeed = applyDeadzone(forwardSpeed, 0.05);
      turnSpeed = applyDeadzone(turnSpeed, 0.05);

      // Calculate left and right motor speeds for tank drive
      double leftSpeed = forwardSpeed + turnSpeed;
      double rightSpeed = forwardSpeed - turnSpeed;
      
      // Set motor speeds in the WestCoastDrive subsystem
      System.out.println(leftSpeed);
      System.out.println(rightSpeed);
      westCoastDrive.setMotorSpeed(leftSpeed, rightSpeed);
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
