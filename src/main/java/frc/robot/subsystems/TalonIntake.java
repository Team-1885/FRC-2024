// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.stream.Stream;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.common.types.input.ELogitech310;
import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;


public class TalonIntake extends Module {
  
private @Getter final ADAM adam = new ADAM(null);

  // initialize devices on the rio can bus
    private static TalonFX INTAKE1 = new TalonFX(0, "rio");
    private static TalonFX INTAKE2 = new TalonFX(1, "rio");

  //is this controller the right one? 
  //is it properly imported? 
  ELogitech310 m_ELogitech310 = new ELogitech310(port:0);

  // users should reuse control requests when possible
  final DutyCycleOut m_leftRequest = new DutyCycleOut(0.0);
  final DutyCycleOut m_rightRequest = new DutyCycleOut(0.0);

  private @Getter RelativeEncoder encoder1, encoder2;

  private ShuffleboardTab tab = Shuffleboard.getTab("===== TALON INTAKE SUBSYSTEM =====");
  private GenericEntry testEntry1 = tab.add("===== SET FEEDER SPEED =====", 0).getEntry();
  private GenericEntry testEntry2 = tab.add("===== SET ROTATION SPEED =====", 0).getEntry(); 

  /** Creates a new TalonsIntake. */
  public TalonIntake() {
    super();
    
    //all of this stuff came from the TalonFX documentation, idk what is usable because it errors when not commented

    //not using joystick so can probably get rid of this section
    /*  retrieve joystick inputs
    var forward = -m_driverJoy.getLeftY();
    var turn = m_driverJoy.getRightX(); */

    //intake not driving? maybe get rid of it
    /* calculate motor outputs, utilizes a "arcade" style of driving;
    // where left Y controls forward and right X controls rotation/turn
    var leftOut = forward + turn;
    var rightOut = forward - turn; */

    //idk about this section, probably needed... maybe
    /* set request to motor controller
    m_leftLeader.setControl(m_leftRequest.withOutput(leftOut));
    m_rightLeader.setControl(m_rightRequest.withOutput(rightOut)); */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runTest(() -> {
          testEntry1.setDouble(INTAKE1.get());
          testEntry2.setDouble(INTAKE2.get());
          REVLibCAN.logFaults(Stream.of(INTAKE1, INTAKE2)); //quick fix for logFaults error doesn't work
          // ... Other periodic tasks
          });
  }


  public void setFeederSpeed(final double feedSpeed) {
    // Setting motor speed using the ".set()" method from the CANSparkMax class
    INTAKE1.set(feedSpeed);
    }

public void setRotaterSpeed(final double rotateSpeed) {
    // Setting motor speed using the ".set()" method from the CANSparkMax class
    INTAKE2.set(rotateSpeed);
    }

public double getFeederSpeed() {
    // Getting motor speed using the ".get()" method from the CANSparkMax class
    return INTAKE1.get();
    }

public double getRotaterSpeed() {
    // Getting motor speed using the ".get()" method from the CANSparkMax class
    return INTAKE2.get();
    }


  public void runTest(final Runnable code) {
      try {
      code.run();
      } catch (Exception e) {
      adam.uncaughtException(Thread.currentThread(), e);
    }
  }
}
