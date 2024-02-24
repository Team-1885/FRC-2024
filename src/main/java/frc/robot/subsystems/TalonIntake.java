// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.stream.Stream;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonIntake extends SubsystemBase{


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
        public TalonIntake() {
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
         * Executes a custom method, running it within a testing environment.
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
    // instantiate motor controllers
TalonFX m_motorLeft = new TalonFX(0);
TalonFX m_motorRight = new TalonFX(1);

// create differentialdrive object for robot control
DifferentialDrive m_diffDrive = new DifferentialDrive(m_motorLeft, m_motorRight);

// instantiate joystick
XboxController m_driverJoy = new XboxController(0);

public void teleopPeriodic() {
   var forward = -m_driverJoy.getLeftY();
   var rot = -m_driverJoy.getRightX();

   m_diffDrive.arcadeDrive(forward, rot);
   // retrieve joystick inputs

var turn = m_driverJoy.getRightX();

// calculate motor outputs, utilizes a "arcade" style of driving;
// where left Y controls forward and right X controls rotation/turn
var leftOut = forward + turn;
var rightOut = forward - turn;

// set request to motor controller
m_leftLeader.setControl(m_leftRequest.withOutput(leftOut));
m_rightLeader.setControl(m_rightRequest.withOutput(rightOut));
}

// initialize devices on the rio can bus
final TalonFX m_leftLeader = new TalonFX(0, "rio");
final TalonFX m_rightLeader = new TalonFX(1, "rio");

// users should reuse control requests when possible
final DutyCycleOut m_leftRequest = new DutyCycleOut(0.0);
final DutyCycleOut m_rightRequest = new DutyCycleOut(0.0);



    private final static int kTimeoutMs = 100;

    public static class Configuration {
        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        // This is factory default.
        public double NEUTRAL_DEADBAND = 0.04;

        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int FORWARD_SOFT_LIMIT = 0;
        public int REVERSE_SOFT_LIMIT = 0;

        public boolean INVERTED = false;
        public boolean SENSOR_PHASE = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;

         public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        // This control frame value seems to need to be something reasonable to avoid the Talon's
        // LEDs behaving erratically.  Potentially try to increase as much as possible.
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 100;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    // Create a CANTalon with the default (out of the box) configuration.
    public static TalonFX createDefaultTalon(int id, Configuration kdefaultconfiguration2) {
        return createDefaultTalon(id, kDefaultConfiguration);
        
    }
    

    /*public static VictorSPX createDefaultVictor(int id) {
        return createVictor(id, kDefaultConfiguration);
    }
    */

    public static TalonFX createPermanentSlaveTalon(int id, int master_id) {
        final TalonFX talon = createTalon(id, kSlaveConfiguration);
        talon.set(master_id);
        return talon;
    }

    private static TalonFX createTalon(int id, Configuration kslaveconfiguration2) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'createTalon'");
    }

    /*public static VictorSPX createPermanentSlaveVictor(int id, TalonSRX master) {
        final VictorSPX victor = createVictor(id, kSlaveConfiguration);
        victor.follow(master);
        return victor;
        
    }
    */
    
    
}
