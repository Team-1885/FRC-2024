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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;


public class PeytonCANLauncher extends SubsystemBase {
  // Creates a new PeytonCANLauncher
  private @Getter final ADAM adam = new ADAM(null);

  private static CANSparkMax REV_LAUNCHER_MOTOR = new CANSparkMax(REVLibCAN.SHOOTER_FEEDER_ID, REVLibCAN.MOTOR_TYPE);

  private @Getter RelativeEncoder shooterEncoder;

  private ShuffleboardTab tab = Shuffleboard.getTab("===== SHOOTER SUBSYSTEM =====");
  private GenericEntry testEntry1 = tab.add("===== SET SHOOTER SPEED =====", 0).getEntry();

  public PeytonCANLauncher() {

    super();
    Stream.of(REV_LAUNCHER_MOTOR).forEach(CANSparkMax::restoreFactoryDefaults);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runTest(() -> {
      testEntry1.setDouble(REV_LAUNCHER_MOTOR.get());
      REVLibCAN.logFaults(Stream.of(REV_LAUNCHER_MOTOR));
      // ... Other periodic tasks
});
  }

  public void reset() {
    runTest(() -> {
            // Resets
            Stream.of(shooterEncoder).forEach(encoder -> encoder.setPosition(0));
            Stream.of(REV_LAUNCHER_MOTOR).forEach(motor -> motor.stopMotor());
    });
  }

  public void setLaunchWheelSpeed(double speed) {
    REV_LAUNCHER_MOTOR.set(speed);
  }

  public double getLaunchWheelSpeed() {
    return REV_LAUNCHER_MOTOR.get();
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

