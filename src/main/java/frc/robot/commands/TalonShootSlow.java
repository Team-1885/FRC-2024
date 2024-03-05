package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NEOFeeder;
import frc.robot.subsystems.TalonIntake;
import static frc.robot.Constants.IntakeConstants;

/*
 * This is an example of creating a command as a class. The base Command class provides a set of methods that your command will override.
 */
public class TalonShootSlow extends Command {
  NEOFeeder mFeeder;

  /** Creates a new TalonFeed. */
  public TalonShootSlow(NEOFeeder pFeeder) {
    // save the feeding system internally
    mFeeder = pFeeder;

    // indicate that this command requires the launcher system
    addRequirements(mFeeder);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the wheels to launching speed
    mFeeder.setIntakeFeeder(-0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // There is nothing we need this command to do on each iteration. You could remove this method and the default blank method of the base class will run.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the scheduler to end the command when the button is released.
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the feeder motor when the command ends.
    mFeeder.stop();
  }
}

