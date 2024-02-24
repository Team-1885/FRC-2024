package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import lombok.Getter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import frc.robot.ADAM;
import java.util.stream.Stream;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

public class TalonIntake extends Module {
    private @Getter final ADAM adam = new ADAM(null);

    // instantiate motor controllers
    TalonFX m_motorLeft = new TalonFX(0);
    TalonFX m_motorRight = new TalonFX(1);

    // instantiate joystick
    XboxController m_XboxController = new XboxController(0);

    // initialize devices on the rio can bus
    final TalonFX m_feeder = new TalonFX(0, "rio");
    final TalonFX m_rotater = new TalonFX(1, "rio");

    // users should reuse control requests when possible
    final DutyCycleOut m_feederRequest = new DutyCycleOut(0.0);
    final DutyCycleOut m_rotaterRequest = new DutyCycleOut(0.0);

    private RelativeEncoder feederEncoder;
    private RelativeEncoder rotaterEncoder;

    public RelativeEncoder getFeederEncoder() { return feederEncoder; }

    public RelativeEncoder getRotateEncoder() { return rotaterEncoder; }


    private ShuffleboardTab tab = Shuffleboard.getTab("===== INTAKE SUBSYSTEM =====");
    private GenericEntry testEntry1 = tab.add("===== SET FEEDER SPEED =====", 0).getEntry();
    private GenericEntry testEntry2 = tab.add("===== SET ROTATION SPEED =====", 0).getEntry();

    private static final TalonIntake instance = new TalonIntake();

        public static TalonIntake getInstance() {
                return instance;
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

    @Override
        public void periodic() { // This method will be called once per scheduler run (usually, once every 20// ms),
                runTest(() -> {
                        // set request to motor controller
                        //m_feeder.setControl(m_feederRequest.withOutput(feederOut));
                        //m_rotater.setControl(m_rotaterRequest.withOutput(rotaterOut));

                });
        }

    // calculate motor outputs, utilizes a "arcade" style of driving;
    // where left Y controls forward and right X controls rotation/turn
    //double leftOut = forward + turn;
    //double rightOut = forward - turn;

    //ADD BUTTON CODE

    public void runTest(final Runnable code) {
                try {
                        code.run();
                } catch (Exception e) {
                        adam.uncaughtException(Thread.currentThread(), e);
                }
    }
    
}
