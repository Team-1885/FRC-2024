package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class NewFalconIntake {

    // instantiate motor controllers
    TalonFX m_motorLeft = new TalonFX(0);
    TalonFX m_motorRight = new TalonFX(1);


    // instantiate joystick
    XboxController m_XboxController = new XboxController(0);

    // initialize devices on the rio can bus
    final TalonFX m_leftLeader = new TalonFX(0, "rio");
    final TalonFX m_rightLeader = new TalonFX(1, "rio");

    // users should reuse control requests when possible
    final DutyCycleOut m_leftRequest = new DutyCycleOut(0.0);
    final DutyCycleOut m_rightRequest = new DutyCycleOut(0.0);

    // FIX ME: ADD BUTTON CODE



    // calculate motor outputs, utilizes a "arcade" style of driving;
    // where left Y controls forward and right X controls rotation/turn
    //double leftOut = forward + turn;
    //double rightOut = forward - turn;

    // set request to motor controller
    m_leftLeader.setControl(m_leftRequest.withOutput(leftOut));
    m_rightLeader.setControl(m_rightRequest.withOutput(rightOut));

}
