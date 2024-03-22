package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rotator extends SubsystemBase {
    TalonFX mRotateMaster = new TalonFX(Constants.IntakeConstants.kRotatorMasterID);
    TalonFX mRotateFollower = new TalonFX(Constants.IntakeConstants.kRotatorFollowerID);
    Slot0Configs slot0Configs = new Slot0Configs();


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

    public void setControl(PositionVoltage pRequest) {
        mRotateMaster.setControl(pRequest);
        mRotateFollower.setControl(pRequest);
    }

    public void applyConfigs(Slot0Configs pConfigs) {
        mRotateMaster.getConfigurator().apply(pConfigs);
        mRotateFollower.getConfigurator().apply(pConfigs);
    }
}
