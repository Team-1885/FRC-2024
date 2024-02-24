package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import lombok.Getter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NewFalconIntake extends Module {

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

    private @Getter RelativeEncoder feederEncoder;
    private @Getter RelativeEncoder rotateEncoder;


    private ShuffleboardTab tab = Shuffleboard.getTab("===== INTAKE SUBSYSTEM =====");
    private GenericEntry testEntry1 = tab.add("===== SET FEEDER SPEED =====", 0).getEntry();
    private GenericEntry testEntry2 = tab.add("===== SET ROTATION SPEED =====", 0).getEntry();



    // calculate motor outputs, utilizes a "arcade" style of driving;
    // where left Y controls forward and right X controls rotation/turn
    //double leftOut = forward + turn;
    //double rightOut = forward - turn;

    //ADD BUTTON CODE

    // set request to motor controller
    m_leftLeader.setControl(m_leftRequest.withOutput(leftOut));
    m_rightLeader.setControl(m_rightRequest.withOutput(rightOut));

}
