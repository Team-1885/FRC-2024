// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.stream.Stream;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;


public class PeytonCANLauncher extends SubsystemBase {
  // Creates a new PeytonCANLauncher
  private @Getter final ADAM adam = new ADAM(null);

  private static CANSparkMax REV_LAUNCHER_MOTOR = new CANSparkMax(REVLibCAN.SHOOTER_1_ID, REVLibCAN.MOTOR_TYPE);
  private static CANSparkMax REV_LAUNCHER_MOTOR2 = new CANSparkMax(REVLibCAN.SHOOTER_2_ID, REVLibCAN.MOTOR_TYPE);

  private @Getter RelativeEncoder launcherEncoder, launcher2Encoder;

  private ShuffleboardTab tab = Shuffleboard.getTab("===== SHOOTER SUBSYSTEM =====");
  private GenericEntry testEntry1 = tab.add("===== SET SHOOTER1 SPEED =====", 0).getEntry();
  private GenericEntry testEntry2 = tab.add("===== SET SHOOTER2 SPEED =====", 0).getEntry();

  public PeytonCANLauncher() {

    super();
    Stream.of(REV_LAUNCHER_MOTOR, REV_LAUNCHER_MOTOR2).forEach(CANSparkMax::restoreFactoryDefaults);
    Stream.of(REV_LAUNCHER_MOTOR, REV_LAUNCHER_MOTOR2).forEach(motor -> motor.setSmartCurrentLimit(30, 35, 100));
    launcherEncoder = REV_LAUNCHER_MOTOR.getEncoder();                                                                                                          
    launcher2Encoder = REV_LAUNCHER_MOTOR2.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runTest(() -> {
      testEntry1.setDouble(REV_LAUNCHER_MOTOR.get());
      testEntry2.setDouble(REV_LAUNCHER_MOTOR2.get());
      REVLibCAN.logFaults(Stream.of(REV_LAUNCHER_MOTOR, REV_LAUNCHER_MOTOR2));
      // ... Other periodic tasks
});
  }

  public void reset() {
    runTest(() -> {
            // Resets
            Stream.of(launcherEncoder, launcher2Encoder).forEach(encoder -> encoder.setPosition(0));
            Stream.of(REV_LAUNCHER_MOTOR, REV_LAUNCHER_MOTOR2).forEach(motor -> motor.stopMotor());
    });
  }

  public void setLaunchSpeed(final double launchSpeed) {
    REV_LAUNCHER_MOTOR.set(launchSpeed);
  }

  public void setLaunch2Speed(final double launch2Speed){
    REV_LAUNCHER_MOTOR2.set(launch2Speed);
  }

  public double getLaunchSpeed() {
    return REV_LAUNCHER_MOTOR.get();
  }

  public double getLaunch2Speed(){
    return REV_LAUNCHER_MOTOR2.get();
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
}

