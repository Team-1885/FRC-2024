// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rotator;


/*
 * This is an example of creating a command as a class. The base Command class provides a set of methods that your command will override.
 */
public class RotateIntakeToDegree extends Command {
  // https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
  Rotator mRotater;

  /** Creates a new LaunchNote. */
  public RotateIntakeToDegree(Rotator pRotater) {
    // save the launcher system internally
    mRotater = pRotater;

    // indicate that this command requires the launcher system
    addRequirements(mRotater);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        Slot0Configs slot0Configs = new Slot0Configs();

        // in init function, set slot 0 gains
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        mRotater.applyConfigs(slot0Configs);

        // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
        final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(80, 160)
        );
        // Final target of 200 rot, 0 rps
        TrapezoidProfile.State m_goal = new TrapezoidProfile.State(100, 0);
        TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
        
        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        
        // calculate the next profile setpoint
        m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
        
        // send the request to the device
        m_request.Position = m_setpoint.position;
        m_request.Velocity = m_setpoint.velocity;
        mRotater.setControl(m_request);
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