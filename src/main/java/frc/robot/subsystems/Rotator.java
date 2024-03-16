package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rotator extends SubsystemBase {
    TalonFX mRotateMaster = new TalonFX(Constants.IntakeConstants.kRotatorMasterID);
    TalonFX mRotateFollower = new TalonFX(Constants.IntakeConstants.kRotatorFollowerID);

    private PIDController intakePID;

    public Rotator() {
        mRotateFollower.setControl(new Follower(mRotateMaster.getDeviceID(), false));
        double kP = 0.01;
        double kI = 0.0;
        double kD = 0.0;
        intakePID = new PIDController(kP, kI, kD);
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

    public void TODO(int pRotations) {
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        mRotateMaster.getConfigurator().apply(slot0Configs);

        // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
        final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(80, 160)
        );
        // Final target of 200 rot, 0 rps
        TrapezoidProfile.State m_goal = new TrapezoidProfile.State(200, 0);
        TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
        
        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        
        // calculate the next profile setpoint
        m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
        
        // send the request to the device
        m_request.Position = m_setpoint.position;
        m_request.Velocity = m_setpoint.velocity;
        mRotateMaster.setControl(m_request);
    }
}
