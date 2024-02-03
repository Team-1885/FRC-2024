// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.ADAM;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PeytonPID;
import lombok.Getter;


@SuppressWarnings("PMD.CommentSize")
public class PeytonPIDCommand extends PIDCommand {

  private @Getter ADAM adam = new ADAM(null);

  private final @Getter PeytonPID peytonPID;

  public PeytonCommand(final PeytonPID peytonPID, PIDController controller, DoubleSupplier measurementSource, DoubleSupplier setpointSource,
  DoubleConsumer useOutput, Subsystem... requirements) {
    super(new PIDController(0, 0, 0), setpointSource, 0, useOutput, requirements);
    getController();
    this.peytonPID = peytonPID;
    addRequirements(peytonPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    System.out.println("==== STARTING PEYTONPIDCOMMAND ====");
    runTest(() -> {

    });
  }

  @Override
  public void execute() {
    super.execute();
    runTest(() -> {
      
      double forwardSpeed = RobotContainer.logitech.getRawAxis(1) * 1;
      double turnSpeed = RobotContainer.logitech.getZ() * 1; 

      forwardSpeed = applyDeadzone(forwardSpeed, 0.05);
      turnSpeed = applyDeadzone(turnSpeed, 0.05);

      double leftSpeed = forwardSpeed + turnSpeed;
      double rightSpeed = forwardSpeed - turnSpeed;

      peytonDriveTrain.setMotorSpeed(leftSpeed, rightSpeed);
    });
  }

  private double applyDeadzone(double value, double deadzone) {
    if (Math.abs(value) < deadzone) {
      return 0.0;
    } else {
      return value;
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    runTest(() -> {

    });
  }

  @Override
  public boolean isFinished() {
    super.isFinished();
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

