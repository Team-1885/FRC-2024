// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The RobotMap class provides a convenient place for teams to hold robot-wide numerical or boolean constants.
 * All constants should be declared globally (i.e. public static). Do not put anything functional in this class.
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to reduce verbosity.
 */
@SuppressWarnings({"PMD.CommentSize", "PMD.LongVariable"}) public final class RobotMap {

  /**
   * ...
   */
  public static final class DriverConstants {
    /**
     * ...
     */
    public static final int D_XBOX_PORT = 0;
    /**
     * ...
     */
    public static final int D_PS4_PORT = 0;
    /**
     * ...
     */
    public static final int D_LOGITECH_PORT = 0;
  }

  /**
   * ...
   */
  public static final class OperatorConstants {
    /**
     * ...
     */
    public static final int O_XBOX_PORT = 0;
    /**
     * ...
     */
    public static final int O_PS4_PORT = 0;
    /**
     * ...
     */
    public static final int O_LOGITECH_PORT = 0;
  }

  /**
   * Button mappings to different FIRST approved controllers 
   * {@linkplain https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html}.
   */
  public static final class ButtonMappings {
    /**
     * ...
     */
    public static final XboxController.Button CONFIG_XBOX_A = XboxController.Button.kA;
    /**
     * ...
     */
    public static final XboxController.Button CONFIG_XBOX_B = XboxController.Button.kB;
    /**
     * ...
     */
    public static final XboxController.Button CONFIG_XBOX_BACK = XboxController.Button.kBack;
    /**
     * ...
     */
    public static final XboxController.Button CONFIG_XBOX_LB = XboxController.Button.kLeftBumper;
    /**
     * ...
     */
    public static final XboxController.Button CONFIG_XBOX_LS = XboxController.Button.kLeftStick;
    /**
     * ...
     */
    public static final XboxController.Button CONFIG_XBOX_RB = XboxController.Button.kRightBumper;
    /**
     * ...
     */
    public static final XboxController.Button CONFIG_XBOX_RS = XboxController.Button.kRightStick;
    /**
     * ...
     */
    public static final XboxController.Button CONFIG_XBOX_START = XboxController.Button.kStart;
    /**
     * ...
     */
    public static final XboxController.Button CONFIG_XBOX_X = XboxController.Button.kX;
    /**
     * ...
     */
    public static final XboxController.Button CONFIG_XBOX_Y = XboxController.Button.kY;

    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_CIRCLE = PS4Controller.Button.kCircle;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_CROSS = PS4Controller.Button.kCross;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_L1 = PS4Controller.Button.kL1;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_L2 = PS4Controller.Button.kL2;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_L3 = PS4Controller.Button.kL3;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_OPTIONS = PS4Controller.Button.kOptions;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_PS = PS4Controller.Button.kPS;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_R1 = PS4Controller.Button.kR1;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_R2 = PS4Controller.Button.kR2;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_R3 = PS4Controller.Button.kR3;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_SHARE = PS4Controller.Button.kShare;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_SQUARE = PS4Controller.Button.kSquare;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_TOUCHPAD = PS4Controller.Button.kTouchpad;
    /**
     * ...
     */
    public static final PS4Controller.Button CONFIG_PS4_TRIANGLE = PS4Controller.Button.kTriangle;

    /**
     * ...
     */
    public static final int LOGITECH_A = 1;
    /**
     * ...
     */
    public static final int LOGITECH_B = 2;
    /**
     * ...
     */
    public static final int LOGITECH_X = 3;
    /**
     * ...
     */
    public static final int LOGITECH_Y = 4;
    /**
     * ...
     */
    public static final int LOGITECH_LB = 5;
    /**
     * ...
     */
    public static final int LOGITECH_RB = 6;
    /**
     * ...
     */
    public static final int LOGITECH_BACK = 7;
    /**
     * ...
     */
    public static final int LOGITECH_START = 8;
    /**
     * ...
     */
    public static final int LOGITECH_LS = 9;
    /**
     * ...
     */
    public static final int LOGITECH_RS = 10;
    /**
     * ...
     */
    public static final int LOGITECH_L_AP = 11;
    /**
     * ...
     */
    public static final int LOGITECH_R_AP = 12;
    /**
     * ...
     */
    public static final int LOGITECH_AX_L_X = 0;
    /**
     * ...
     */
    public static final int LOGITECH_AX_L_Y = 1;
    /**
     * ...
     */
    public static final int LOGITECH_AX_LT = 2;
    /**
     * ...
     */
    public static final int LOGITECH_AX_RT = 3;
    /**
     * ...
     */
    public static final int LOGITECH_AX_R_X = 4;
    /**
     * ...
     */
    public static final int LOGITECH_AX_R_Y = 5;
  }
}
