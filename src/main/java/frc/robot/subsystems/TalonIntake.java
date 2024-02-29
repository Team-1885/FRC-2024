package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonIntake extends SubsystemBase {
  CANSparkMax mIntakeFeeder = new CANSparkMax(3, MotorType.kBrushless);
  TalonFX mRotateMaster = new TalonFX(1);
  TalonFX mRotateFollower = new TalonFX(2);

  /** Creates a new Launcher. */
  public TalonIntake() {
    mIntakeFeeder.setSmartCurrentLimit(70);
    mRotateFollower.setControl(new Follower(mRotateMaster.getDeviceID(), false));
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  public Command getTalonIntakeCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setIntakeFeeder(0.7);
          setIntakeRotater(0.5);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        }); 
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setIntakeFeeder(double speed) {
    mIntakeFeeder.set(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setIntakeRotater(double speed) {
    mRotateMaster.set(speed);
    mRotateFollower.set(-speed);
  }

  public double getMasterSpeed() {
    return mRotateMaster.get();
  }

  public double getFollowerSpeed() {
    return mRotateFollower.get();
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    mIntakeFeeder.set(0);
    mRotateMaster.set(0);
    mRotateFollower.set(0);
  }
}