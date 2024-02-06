// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.stream.Stream;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;

//Purpose of this class: To create a subsystem with two spinning motors. The top motor spins inverted
//to the bottom motor. So while the bottom is spinning clockwise, the bottom is anti-clockwise
//The top motor is represented by Motor2, while the bottom is represented by Motor1.

public class Climber extends Module {

    private static CANSparkMax Motor1 = new CANSparkMax(20, REVLibCAN.MOTOR_TYPE); // PLACEHOLDER ID'S AND TYPE
    private static CANSparkMax Motor2 = new CANSparkMax(21, REVLibCAN.MOTOR_TYPE); // PLACEHOLDER ID'S AND TYPE
    private @Getter ADAM adam = new ADAM(null);
    private @Getter RelativeEncoder topMotor,
            bottomMotor;
    private NetworkTable mTable;
    private final Field2d mField = new Field2d();
    private ShuffleboardTab tab = Shuffleboard.getTab("Climber");
    private GenericEntry MotorOne = tab.add("Left Master", 0)
            .getEntry();
    private GenericEntry MotorTwo = tab.add("Left Follower", 0)
            .getEntry();

    // Creates a new climber and motors, basically all the boring stuff with
    // creating new motors.
    // Here's where motors become inverted and rates (MotorSpeed, Limits, Encoders)
    // get set.
    public Climber() {
        super();
        Stream.of(Motor1, Motor2)
                .forEach(CANSparkMax::restoreFactoryDefaults);

        // THIS IS WHERE MOTOR 2 IS SET TO FOLLOW MOTOR 1
        final Map<CANSparkMax, CANSparkMax> masterFollowerMap = Map.of(
                Motor1, Motor2);
        masterFollowerMap.forEach((master, follower) -> follower.follow(master));

        Stream.of(Motor1)
                .forEach(motor -> motor.setInverted(false));

        Stream.of(Motor2)
                .forEach(motor -> motor.setInverted(true));

        Stream.of(Motor1, Motor2)
                .forEach(motor -> motor.setIdleMode(CANSparkMax.IdleMode.kCoast));

        Stream.of(Motor1, Motor2)
                .forEach(motor -> motor.setSmartCurrentLimit(30, 35, 100));

        bottomMotor = Motor1.getEncoder();
        topMotor = Motor2.getEncoder();

        Stream.of(Motor1, Motor2)
                .forEach(motor -> motor.setClosedLoopRampRate(1));

        Stream.of(Motor1, Motor2)
                .forEach(motor -> motor.setControlFramePeriodMs(1));

        Stream.of(Motor1, Motor2)
                .forEach(CANSparkMax::burnFlash);

    }

    public void setMotorSpeed() {
        // Setting motor speed using the ".set()" method from the CANSparkMax class
        Motor1.set(0.1); // TODO: need to test Michael
        Motor2.set(0.1);
    }

    public void reverse() {
        Motor1.set(-0.5);
        Motor2.set(-0.5);
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run (usually, once every 20
                             // ms),
        runTest(() -> {
            REVLibCAN.logFaults(Stream.of(Motor1, Motor2));


            MotorOne.setDouble(Motor1.get());
            MotorTwo.setDouble(Motor2.get());
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
            Stream.of(bottomMotor, topMotor)
                    .forEach(encoder -> encoder.setPosition(0));
            Stream.of(Motor1, Motor2)
                    .forEach(motor -> motor.stopMotor());
        });
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
