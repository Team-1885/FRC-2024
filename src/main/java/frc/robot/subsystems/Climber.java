// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  TalonSRX climberR = new TalonSRX(11);
  TalonSRX climberL = new TalonSRX(12);
  
  /** Creates a new Climber. */
  public Climber() {
    climberR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    climberL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void stop() {
    climberR.set(ControlMode.PercentOutput, 0.0);
    climberL.set(ControlMode.PercentOutput, 0.0);

  }

  public void setClimberSpeed(double speedL, double speedR) {
    climberR.set(ControlMode.PercentOutput, Constants.ClimberConstants.speedRight * speedR);
    climberL.set(ControlMode.PercentOutput, -1 * Constants.ClimberConstants.speedLeft * speedL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double posL = climberL.getSelectedSensorPosition();
    double posR = climberR.getSelectedSensorPosition();
    SmartDashboard.putNumber("L: ", posL);
    SmartDashboard.putNumber("R: ", posR);

    double velL = climberL.getSelectedSensorVelocity();
    double velR = climberR.getSelectedSensorVelocity();
    SmartDashboard.putNumber("L: ", velL);
    SmartDashboard.putNumber("R: ", velR);
  }

}