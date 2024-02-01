// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.stream.Stream;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;

//remember to import into RobotContainer

//added some PID stuff, don't know yet what is relevant and what isn't

@SuppressWarnings("PMD.CommentSize")
public class PeytonPID extends PIDSubsystem {

  private @Getter final ADAM adam = new ADAM(null);

  private static CANSparkMax Motor = new CANSparkMax(REVLibCAN.L_MASTER_ID, REVLibCAN.MOTOR_TYPE); //left master motor
                        
  private @Getter RelativeEncoder leftEncoder, rightEncoder;

  private ShuffleboardTab tab = Shuffleboard.getTab("===== Peyton Drive Train =====");
  private GenericEntry testEntry = tab.add("===== SET MOTOR SPEED =====", 0).getEntry();   
  
  
  /** Creates a new PeytonPID. */
  public PeytonPID() {

    super(new PIDController(0.5, 0.05, 0.1)); //these terms can/will be changed
    getController();

    // The subsystem will track to a setpoint of 10.
    setSetpoint(10);

    enable();
    disable();

    Stream.of(Motor).forEach(CANSparkMax::restoreFactoryDefaults);
    final Map<CANSparkMax, CANSparkMax> masterFollowerMap = Map.of();
    masterFollowerMap.forEach((master, follower) -> follower.follow(master));
    Motor(setInverted(true));
    Stream.of(Motor).forEach(motor -> motor.setIdleMode(CANSparkMax.IdleMode.kCoast));
    Stream.of(Motor)
                .forEach(motor -> motor.setSmartCurrentLimit(30, 35, 100));
                leftEncoder = Motor.getEncoder();
    Stream.of(Motor).forEach(motor -> motor.setClosedLoopRampRate(1));
    Stream.of(Motor).forEach(motor -> motor.setControlFramePeriodMs(1));
    Stream.of(Motor).forEach(CANSparkMax::burnFlash);

  }

  private void Motor(Object setInverted) {
  }

  private Object setInverted(boolean b) {
    return null;
  }

  @Override
  public void periodic() {
    runTest(() -> {
      testEntry.setDouble(Motor.get());
      REVLibCAN.logFaults(Motor);
      });

      // This method will be called once per scheduler run
    super.periodic();
    getMeasurement();
    useOutput(0, 0);

}
  public void reset() {
    runTest(() -> {
      Stream.of(leftEncoder, rightEncoder).forEach(encoder -> encoder.setPosition(0));
      Stream.of(Motor).forEach(motor -> motor.stopMotor());
      });
}

public void setMotorSpeed(final double leftSpeed, final double rightSpeed) {
  Motor.set(leftSpeed * 0.5);
}

public double getMotorSpeed() {
  return Motor.get();
}

public void debugSubsystem() {
  runTest(() -> periodic());
  runTest(() -> reset());
}

public void runTest(final Runnable code) {
  try {
          code.run();
  } catch (Exception e) {
          adam.uncaughtException(Thread.currentThread(), e);
  }
}

@Override
protected void useOutput(double output, double setpoint) {
  // autogenerated method
}

@Override
protected double getMeasurement() {
  // autogenerated method
  return 0;
}
}


