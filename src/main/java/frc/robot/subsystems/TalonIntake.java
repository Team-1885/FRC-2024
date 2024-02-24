// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.stream.Stream;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;

public class TalonIntake extends Module {
  
private final ADAM adam = new ADAM(null);

  //is this controller the right one? 
  //why isn't the import working
  XboxController m_XboxController = new XboxController(0);

  // initialize devices on the rio can bus
  final TalonFX INTAKE1 = new TalonFX(0, "rio");
  final TalonFX INTAKE2 = new TalonFX(1, "rio");

  // users should reuse control requests when possible
  final DutyCycleOut m_INTAKE1Request = new DutyCycleOut(0.0);
  final DutyCycleOut m_INTAKE2Request = new DutyCycleOut(0.0);

  private RelativeEncoder intake1Encoder, intake2Encoder;

  private ShuffleboardTab tab = Shuffleboard.getTab("===== TALON INTAKE SUBSYSTEM =====");
  private GenericEntry testEntry1 = tab.add("===== SET FEEDER SPEED =====", 0).getEntry();
  private GenericEntry testEntry2 = tab.add("===== SET ROTATION SPEED =====", 0).getEntry(); 

  private static final TalonIntake instance = new TalonIntake();

      public static TalonIntake getInstance() {
        return instance;
      }
      
  /** Creates a new TalonsIntake. */
  public TalonIntake() {
    super();

    //borrowed this code from Katelyn because I couldn't figure out what to put there

    //start with factory-default configs
        var currentConfigs = new MotorOutputConfigs();

        // The left motor is CCW+
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        INTAKE1.getConfigurator().apply(currentConfigs);

        // The right motor is CW+
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        INTAKE2.getConfigurator().apply(currentConfigs);
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


    //intake not driving? maybe can get rid of it
    /* calculate motor outputs, utilizes a "arcade" style of driving;
    // where left Y controls forward and right X controls rotation/turn
    var 1Out = forward + turn;
    var 2Out = forward - turn; */

  public void runTest(final Runnable code) {
      try {
      code.run();
      } catch (Exception e) {
      adam.uncaughtException(Thread.currentThread(), e);
    }
  }
}
