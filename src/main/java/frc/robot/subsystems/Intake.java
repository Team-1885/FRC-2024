package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax mIntakeFeeder = new CANSparkMax(3, MotorType.kBrushless);

    public Intake() {
        mIntakeFeeder.setSmartCurrentLimit(80);
    }

    public Command getTalonIntakeCommand() {
        // The startEnd helper method takes a method to call when the command is
        // initialized and one to
        // call when it ends
        return this.startEnd(
                // When the command is initialized, set the wheels to the intake speed values
                () -> {
                    setIntakeFeeder(0.7);
                },
                // When the command stops, stop the wheels
                () -> {
                    stop();
                });
    }

    // An accessor method to set the speed (technically the output percentage) of
    // the launch wheel
    public void setIntakeFeeder(double speed) {
        mIntakeFeeder.set(speed);
    }

    // A helper method to stop both wheels. You could skip having a method like this
    // and call the
    // individual accessors with speed = 0 instead
    public void stop() {
        mIntakeFeeder.set(0);
    }
}
