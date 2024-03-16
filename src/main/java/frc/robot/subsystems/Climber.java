// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  CANSparkMax climber1 = new CANSparkMax(Constants.ClimberConstants.climber1ID, MotorType.kBrushed);
  CANSparkMax climber2 = new CANSparkMax(Constants.ClimberConstants.climber2ID, MotorType.kBrushed);
  
  /** Creates a new Climber. */
  public Climber() {}

  public Command getClimberCommand() {
        // The startEnd helper method takes a method to call when the command is
        // initialized and one to
        // call when it ends
        return this.startEnd(
                // When the command is initialized, set the wheels to the intake speed values
                () -> {
                    climber1.set(0.5);
                    climber2.set(0.5);
                },
                // When the command stops, stop the wheels
                () -> {
                    stop();
                });
    }

  public void stop() {
    climber1.set(0.0);
    climber2.set(0.0);
  }

  public void setClimberSpeed(double speed) {
    climber1.set(speed);
    climber2.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}