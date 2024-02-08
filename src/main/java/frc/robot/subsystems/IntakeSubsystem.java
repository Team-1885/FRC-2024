// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Stream;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ADAM;
import frc.robot.RobotMap;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;


/**
 * Intake Subsystem.
 */
@SuppressWarnings("PMD.CommentSize")
public class IntakeSubsystem extends SubsystemBase {
        private @Getter final ADAM adam = new ADAM(null);

        // Creates a CANSparkMax motor, inheriting physical constants from the {@link#REVLibCAN} helper class.
        private static CANSparkMax REV_INTAKE_FEEDER = new CANSparkMax(REVLibCAN.INTAKE_FEEDER_ID, REVLibCAN.MOTOR_TYPE);

        // Creates a CANSparkMax motor, inheriting physical constants from the {@link#REVLibCAN} helper class.
        private static CANSparkMax REV_INTAKE_ROTATER = new CANSparkMax(REVLibCAN.INTAKE_ROTATER_ID, REVLibCAN.MOTOR_TYPE);
        /**
         * Lorem Ipsum.
         */
        private @Getter RelativeEncoder feederEncoder, rotateEncoder;

        private ShuffleboardTab tab = Shuffleboard.getTab("===== INTAKE SUBSYSTEM =====");
        private GenericEntry testEntry1 = tab.add("===== SET FEEDER SPEED =====", 0).getEntry();
        private GenericEntry testEntry2 = tab.add("===== SET ROTATION SPEED =====", 0).getEntry();

        /** Constructor for the Subsystem */
        public IntakeSubsystem() {
                /**
                 * Low-level configurations for the hardware objects
                 */
                super();
                Stream.of(REV_INTAKE_FEEDER, REV_INTAKE_ROTATER).forEach(CANSparkMax::restoreFactoryDefaults);
                /*Stream.of(REV_0xM1, REV_0xF1)
                                .forEach(motor -> motor.setInverted(false));
                Stream.of(REV_0xM2, REV_0xF2)
                                .forEach(motor -> motor.setInverted(true));
                Stream.of(REV_0xM1, REV_0xF1, REV_0xM2, REV_0xF2)
                                .forEach(motor -> motor.setIdleMode(CANSparkMax.IdleMode.kCoast)); */
                Stream.of(REV_INTAKE_FEEDER, REV_INTAKE_ROTATER).forEach(motor -> motor.setSmartCurrentLimit(30, 35, 100));
                feederEncoder = REV_INTAKE_FEEDER.getEncoder();
                rotateEncoder = REV_INTAKE_ROTATER.getEncoder();
                Stream.of(REV_INTAKE_FEEDER, REV_INTAKE_ROTATER).forEach(motor -> motor.setClosedLoopRampRate(0.5));
                Stream.of(REV_INTAKE_FEEDER, REV_INTAKE_ROTATER).forEach(motor -> motor.setOpenLoopRampRate(0.5));
                Stream.of(REV_INTAKE_FEEDER, REV_INTAKE_ROTATER).forEach(motor -> motor.setControlFramePeriodMs(1));
                Stream.of(REV_INTAKE_FEEDER, REV_INTAKE_ROTATER).forEach(CANSparkMax::burnFlash);

        }

        @Override
        public void periodic() { // This method will be called once per scheduler run (usually, once every 20 ms),
                runTest(() -> {
                        testEntry1.setDouble(REV_INTAKE_FEEDER.get());
                        testEntry2.setDouble(REV_INTAKE_ROTATER.get());
                        REVLibCAN.logFaults(Stream.of(REV_INTAKE_FEEDER, REV_INTAKE_ROTATER));
                        // ... Other periodic tasks
                });
        }

        /**
         * Executes a custom method, running =it within a testing environment.
         *
         * @see #runTest(Runnable)
         */
        public void reset() {
                runTest(() -> {
                        // Resets
                        Stream.of(feederEncoder, rotateEncoder).forEach(encoder -> encoder.setPosition(0));
                        Stream.of(REV_INTAKE_FEEDER, REV_INTAKE_ROTATER).forEach(motor -> motor.stopMotor());
                });
        }

        public void setFeederSpeed(final double feedSpeed) {
                // Setting motor speed using the ".set()" method from the CANSparkMax class
                REV_INTAKE_FEEDER.set(feedSpeed);
        }

        public void setRotaterSpeed(final double rotateSpeed) {
                // Setting motor speed using the ".set()" method from the CANSparkMax class
                REV_INTAKE_ROTATER.set(rotateSpeed);
        }

        public double getFeederSpeed() {
                // Getting motor speed using the ".get()" method from the CANSparkMax class
                return REV_INTAKE_FEEDER.get();
        }

        public double getRotaterSpeed() {
                // Getting motor speed using the ".get()" method from the CANSparkMax class
                return REV_INTAKE_ROTATER.get();
                }

        /**
         * Executes custom testing and validation methods in a controlled environment.
         * Any exceptions thrown during execution are caught and logged.
         *
         * @see #runTest(Runnable)
         */
        public void debugSubsystem() {
                runTest(() -> periodic());
                runTest(() -> reset());
        }

        /**
         * Runs the code as a task; if an exception occurs, it is caught, and an
         * uncaught exception is passed to the default handler for the current thread.
         *
         * @param code The runnable task to be executed.
         */
        public void runTest(final Runnable code) {
                try {
                        code.run();
                } catch (Exception e) {
                        adam.uncaughtException(Thread.currentThread(), e);
                }
        }

}
