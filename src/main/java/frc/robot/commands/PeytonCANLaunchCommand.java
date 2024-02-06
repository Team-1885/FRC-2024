// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PeytonCANLauncher;
import lombok.Getter;
//don't have any imports, idk what I need to import because no errors are showing

@SuppressWarnings("PMD.CommentSize")
public class PeytonCANLaunchCommand extends CommandBase {

private @Getter ADAM adam = new ADAM(null);

private final @Getter PeytonCANLauncher peytonCANLauncher;

  // Creates a new PeytonCANLaunchCommand.
  public PeytonCANLaunchCommand(final PeytonCANLauncher peytonCANLauncher) {
    super();
    // Use addRequirements() here to declare subsystem dependencies.
    this.peytonCANLauncher = peytonCANLauncher;
    addRequirements(peytonCANLauncher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========== STARTING LAUNCHER COMMAND ==========");
    runTest(() -> {

    });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runTest(() -> {
      double launchSpeed = 0;

      //input from buttons
      if(ELogitech310.A_BTN.isButton())
      {
        launchSpeed = 0.5;
      }

      launcherSubsystem.setLauncherSpeed(launchSpeed);
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