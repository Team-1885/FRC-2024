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
import edu.wpi.first.wpilibj.XBoxController;
import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;


public class TalonIntake extends Module {
  
private @Getter final ADAM adam = new ADAM(null);

  TalonFX INTAKEMotor1 = new TalonFX(deviceId:0);
  TalonFX INTAKEMotor2 = new TalonFX(deviceId:1);

  //is this controller the right one? 
  //why isn't the import working
  XBoxController m_XBoxController = new XBoxController(port:0);

  // initialize devices on the rio can bus
  final TalonFX INTAKE1 = new TalonFX(0, "rio");
  final TalonFX INTAKE2 = new TalonFX(1, "rio");

  // users should reuse control requests when possible
  final DutyCycleOut m_INTAKE1Request = new DutyCycleOut(0.0);
  final DutyCycleOut m_INTAKE2Request = new DutyCycleOut(0.0);

  private @Getter RelativeEncoder intake1Encoder, intake2Encoder;

  private ShuffleboardTab tab = Shuffleboard.getTab("===== TALON INTAKE SUBSYSTEM =====");
  private GenericEntry testEntry1 = tab.add("===== SET FEEDER SPEED =====", 0).getEntry();
  private GenericEntry testEntry2 = tab.add("===== SET ROTATION SPEED =====", 0).getEntry(); 

  /** Creates a new TalonsIntake. */
  public TalonIntake() {
    super();
   
    //figure out what to put here

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runTest(() -> {
          //set request to motor controller
          /* INTAKE1.setControl(m_INTAKE1Request.withOutput(1Out));
          INTAKE2.setControl(m_INTAKE2Request.withOutput(2Out)); */
          });
  }


  //i think I'm mixing up talons and CANSparkMax methods- figure out if I need these next 4

  /* public void setFeederSpeed(final double feedSpeed) {
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
    } */

    //intake not driving? maybe can get rid of it
    /* calculate motor outputs, utilizes a "arcade" style of driving;
    // where left Y controls forward and right X controls rotation/turn
    var leftOut = forward + turn;
    var rightOut = forward - turn; */

  public void runTest(final Runnable code) {
      try {
      code.run();
      } catch (Exception e) {
      adam.uncaughtException(Thread.currentThread(), e);
    }
  }
}
