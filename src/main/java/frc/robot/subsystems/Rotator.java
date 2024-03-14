package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotator extends SubsystemBase {
    TalonFX mRotateMaster = new TalonFX(11);
    TalonFX mRotateFollower = new TalonFX(24);

    public Rotator() {
        mRotateFollower.setControl(new Follower(mRotateMaster.getDeviceID(), false));
    }

    public Command getRotatorCommand() {
        // The startEnd helper method takes a method to call when the command is
        // initialized and one to
        // call when it ends
        return this.startEnd(
                // When the command is initialized, set the wheels to the intake speed values
                () -> {
                    setIntakeRotater(0.5);
                },
                // When the command stops, stop the wheels
                () -> {
                    stop();
                });
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

    // A helper method to stop both wheels. You could skip having a method like this
    // and call the individual accessors with speed = 0 instead
    public void stop() {
        mRotateMaster.set(0);
        mRotateFollower.set(0);
    }

    public void turnToDegree() {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        mRotateMaster.getConfigurator().apply(slot0Configs);

        final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(80, 160));
        TrapezoidProfile.State m_goal = new TrapezoidProfile.State(200, 0);
        TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

        m_request.Position = m_setpoint.position;
        m_request.Velocity = m_setpoint.velocity;
        mRotateMaster.setControl(m_request);
    }
}
