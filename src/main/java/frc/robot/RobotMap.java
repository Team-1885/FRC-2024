// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.hardware.vendors.firstparties.ABC;

@SuppressWarnings({ "PMD.CommentSize", "PMD.LongVariable" })
public final class RobotMap {

  public static final class DriveConstants {
    public static final double kTrackWidthMeters = 0.530225;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically for *your* robot's drive. The Robot Characterization Toolsuite provides a convenient tool for obtaining these values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    public static final double kGearRatio = 8.46;
    public static final double kWheelRadiusInches = 3;

    public static final double kLinearDistanceConversionFactor = (ABC.inches_to_meters(1 / (kGearRatio * 2 * Math.PI * ABC.inches_to_meters(kWheelRadiusInches)) * 10));
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class DriverConstants {
    public static final int D_XBOX_PORT = 0;
    public static final int D_PS4_PORT = 0;
    public static final int D_LOGITECH_PORT = 0;
  }

  public static final class OperatorConstants {
    public static final int O_XBOX_PORT = 0;
    public static final int O_PS4_PORT = 0;
    public static final int O_LOGITECH_PORT = 0;
  }

  /**
   * Button mappings to different FIRST approved controllers
   * {@linkplain https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html}.
   */
  public static final class ButtonMappings {

    public static final XboxController.Button CONFIG_XBOX_A = XboxController.Button.kA;
    public static final XboxController.Button CONFIG_XBOX_B = XboxController.Button.kB;
    public static final XboxController.Button CONFIG_XBOX_BACK = XboxController.Button.kBack;
    public static final XboxController.Button CONFIG_XBOX_LB = XboxController.Button.kLeftBumper;
    public static final XboxController.Button CONFIG_XBOX_LS = XboxController.Button.kLeftStick;
    public static final XboxController.Button CONFIG_XBOX_RB = XboxController.Button.kRightBumper;
    public static final XboxController.Button CONFIG_XBOX_RS = XboxController.Button.kRightStick;
    public static final XboxController.Button CONFIG_XBOX_START = XboxController.Button.kStart;
    public static final XboxController.Button CONFIG_XBOX_X = XboxController.Button.kX;
    public static final XboxController.Button CONFIG_XBOX_Y = XboxController.Button.kY;

    public static final PS4Controller.Button CONFIG_PS4_CIRCLE = PS4Controller.Button.kCircle;
    public static final PS4Controller.Button CONFIG_PS4_CROSS = PS4Controller.Button.kCross;
    public static final PS4Controller.Button CONFIG_PS4_L1 = PS4Controller.Button.kL1;
    public static final PS4Controller.Button CONFIG_PS4_L2 = PS4Controller.Button.kL2;
    public static final PS4Controller.Button CONFIG_PS4_L3 = PS4Controller.Button.kL3;
    public static final PS4Controller.Button CONFIG_PS4_OPTIONS = PS4Controller.Button.kOptions;
    public static final PS4Controller.Button CONFIG_PS4_PS = PS4Controller.Button.kPS;
    public static final PS4Controller.Button CONFIG_PS4_R1 = PS4Controller.Button.kR1;
    public static final PS4Controller.Button CONFIG_PS4_R2 = PS4Controller.Button.kR2;
    public static final PS4Controller.Button CONFIG_PS4_R3 = PS4Controller.Button.kR3;
    public static final PS4Controller.Button CONFIG_PS4_SHARE = PS4Controller.Button.kShare;
    public static final PS4Controller.Button CONFIG_PS4_SQUARE = PS4Controller.Button.kSquare;
    public static final PS4Controller.Button CONFIG_PS4_TOUCHPAD = PS4Controller.Button.kTouchpad;
    public static final PS4Controller.Button CONFIG_PS4_TRIANGLE = PS4Controller.Button.kTriangle;

    public static final int LOGITECH_A = 1;
    public static final int LOGITECH_B = 2;
    public static final int LOGITECH_X = 3;
    public static final int LOGITECH_Y = 4;
    public static final int LOGITECH_LB = 5;
    public static final int LOGITECH_RB = 6;
    public static final int LOGITECH_BACK = 7;
    public static final int LOGITECH_START = 8;
    public static final int LOGITECH_LS = 9;
    public static final int LOGITECH_RS = 10;
    public static final int LOGITECH_L_AP = 11;
    public static final int LOGITECH_R_AP = 12;
    public static final int LOGITECH_AX_L_X = 0;
    public static final int LOGITECH_AX_L_Y = 1;
    public static final int LOGITECH_AX_LT = 2;
    public static final int LOGITECH_AX_RT = 3;
    public static final int LOGITECH_AX_R_X = 4;
    public static final int LOGITECH_AX_R_Y = 5;
  }
}
