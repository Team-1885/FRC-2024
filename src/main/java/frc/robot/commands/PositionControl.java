// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rotator;

/*
 * This is an example of creating a command as a class. The base Command class provides a set of methods that your command will override.
 */
public class PositionControl extends Command {

    Rotator mRotater;

  /** Creates a new LaunchNote. */
  public PositionControl(Rotator pRotater) {
    // save the launcher system internally
    mRotater = pRotater;

    // indicate that this command requires the launcher system
    addRequirements(mRotater);
  }

    // The initialize method is called when the command is initially scheduled.
    @Override
    public void initialize() {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.1;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.1;

        mRotater.applyConfigs(slot0Configs);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        mRotater.setControl(m_request.withPosition(30)); //10 Rotations
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
        // Stop the wheels when the command ends.
        mRotater.stop();
    }
}