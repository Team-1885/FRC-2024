// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import frc.robot.hardware.vendors.thirdparties.ctre.CTRETalonFX;


/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonIntake extends SubsystemBase{


private final ADAM adam = new ADAM(null);
    // instantiate motor controllers
TalonFX m_motorLeft = new TalonFX(0);
TalonFX m_motorRight = new TalonFX(1);
        // initialize devices on the rio can bus
final TalonFX m_leftLeader = new TalonFX(0, "rio");
final TalonFX m_rightLeader = new TalonFX(1, "rio");

// users should reuse control requests when possible
final DutyCycleOut m_leftRequest = new DutyCycleOut(0.0);
final DutyCycleOut m_rightRequest = new DutyCycleOut(0.0);

       
private RelativeEncoder feederEncoder, rotateEncoder;

        private ShuffleboardTab tab = Shuffleboard.getTab("===== INTAKE SUBSYSTEM =====");
        private GenericEntry testEntry1 = tab.add("===== SET FEEDER SPEED =====", 0).getEntry();
        private GenericEntry testEntry2 = tab.add("===== SET ROTATION SPEED =====", 0).getEntry();

        private static final TalonIntake instance = new TalonIntake();
        private static final CoreTalonFX m_feeder = null;
        private static final CoreTalonFX m_rotater = null;

        public static TalonIntake getInstance() {
                return instance;
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
        public TalonIntake(){
        // start with factory-default configs
        var currentConfigs = new MotorOutputConfigs();

        // The left motor is CCW+
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_feeder.getConfigurator().apply(currentConfigs);

        // The right motor is CW+
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_rotater.getConfigurator().apply(currentConfigs);

    }

        private Object reset() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'reset'");
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
    public void setRotaterSpeed(double rotateSpeed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRotaterSpeed'");
    }
    public void setFeederSpeed(double feedSpeed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFeederSpeed'");
    }

    /*public static VictorSPX createPermanentSlaveVictor(int id, TalonSRX master) {
        final VictorSPX victor = createVictor(id, kSlaveConfiguration);
        victor.follow(master);
        return victor;
        
    }
    */
    
    
}
