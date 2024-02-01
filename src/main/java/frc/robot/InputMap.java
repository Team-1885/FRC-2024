// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.flybotix.hfr.codex.CodexOf;

import edu.wpi.first.wpilibj.Joystick;
import frc.common.types.input.ELogitech310;
import lombok.Getter;

/**
 * Manages controller input and bindings for the Operator Interface (OI).
 */

@SuppressWarnings("PMD.CommentSize")
public class InputMap {

  /**
   * ...
   */
  private @Getter static ADAM adam = new ADAM(null);

  /**
   * ...
   */
  private @Getter Joystick driverLGTCtrlr;

  /**
   * ...
   */
  private @Getter Joystick operatorLGTCtrlr;

  /**
   * Constructs a new instance of the OI class.
   * Initializes driver and operator controllers.
   */
  public InputMap() {
    runTest(() -> {
      driverLGTCtrlr = new Joystick(RobotMap.DriverConstants.D_LOGITECH_PORT);
      operatorLGTCtrlr = new Joystick(RobotMap.OperatorConstants.O_LOGITECH_PORT);
    });
  }

  /**
   * ================================
   * // ...
   * ================================
   */
  public static class DRIVER {
    public static final ELogitech310 TURN_AXIS = ELogitech310.RIGHT_X_AXIS,
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,
        SNAIL_MODE = ELogitech310.RIGHT_TRIGGER_AXIS, // TODO Experiment with increased or decreased acceleration rates
                                                      // for snail
        ACTIVATE_CLIMB = ELogitech310.START,
        MID_RUNG = ELogitech310.L_BTN,
        TARGET_LOCK = ELogitech310.LEFT_TRIGGER_AXIS;
  }

  /**
   * ================================
   * // ...
   * ================================
   */
  public static class OPERATOR {
    public static final ELogitech310 RETRACT_INTAKE = ELogitech310.RIGHT_TRIGGER_AXIS,
        EXTEND_INTAKE = ELogitech310.LEFT_TRIGGER_AXIS,
        REVERSE_ROLLERS = ELogitech310.B_BTN,
        SPIN_FEEDER = ELogitech310.X_BTN,
        REVERSE_FEEDER = ELogitech310.Y_BTN,
        RELEASE_BALLS = ELogitech310.DPAD_LEFT,
        SHOOT_CARGO = ELogitech310.R_BTN,
        STAGE_BALLS = ELogitech310.A_BTN,
        PLACE_CARGO = ELogitech310.L_BTN,

        INCREASE_FEEDER_SPEED = ELogitech310.DPAD_UP,
        DECREASE_FEEDER_SPEED = ELogitech310.DPAD_DOWN;;
  }

  public static class HANGER {
    public static final ELogitech310 SPIN_DOUBLE = ELogitech310.RIGHT_TRIGGER_AXIS,
        SPIN_SINGLE = ELogitech310.LEFT_TRIGGER_AXIS,
        HIGH_RUNG = ELogitech310.L_BTN,
        TRAVERSAL_RUNG = ELogitech310.R_BTN,
        BALANCE_CLIMBER = ELogitech310.START,
        SET_COAST = ELogitech310.BACK,
        CLAMP_DOUBLE = ELogitech310.DPAD_UP,
        RELEASE_DOUBLE = ELogitech310.DPAD_DOWN,
        CLAMP_SINGLE = ELogitech310.Y_BTN,
        RELEASE_SINGLE = ELogitech310.A_BTN,

        GRAB_MID = ELogitech310.L_BTN,
        CONFIRM_CLAMPED_ON_HIGH_BAR = ELogitech310.R_BTN,
        CONFIRM_CLAMPED_ON_TRAVERSAL_RELEASE_HIGH = ELogitech310.X_BTN;
  }

  public enum EDriveData implements CodexOf<Double> {
    // Sensor inputs
    L_ACTUAL_POS_FT, R_ACTUAL_POS_FT,
    L_DESIRED_POS_FT, R_DESIRED_POS_FT,
    L_ACTUAL_VEL_FT_s, R_ACTUAL_VEL_FT_s,
    L_ACTUAL_VEL_RPM, R_ACTUAL_VEL_RPM,
    LEFT_CURRENT, RIGHT_CURRENT,
    LEFT_VOLTAGE, RIGHT_VOLTAGE,
    IS_CURRENT_LIMITING,
    DESIRED_TURN_ANGLE_deg,
    DESIRED_TURN_PCT, DESIRED_THROTTLE_PCT,

    // Ramsete odometry enums
    GET_X_OFFSET, GET_Y_OFFSET,
    GET_X_OFFSET_METERS, GET_Y_OFFSET_METERS,

    ACTUAL_HEADING_RADIANS,
    ACTUAL_HEADING_DEGREES,

    NEUTRAL_MODE,
    STATE,
    L_DESIRED_VEL_FT_S,
    R_DESIRED_VEL_FT_S,
    X_ACTUAL_ODOMETRY_METERS,
    Y_ACTUAL_ODOMETRY_METERS,
    X_DESIRED_ODOMETRY_METERS,
    Y_DESIRED_ODOMETRY_METERS,
    ;
  }

  public class ENeutralMode {

  }

  /**
   * Executes controller access simulation for testing purposes.
   * Calls controller retrieval methods within a testing environment.
   */
  public void debugClass() {
    // ...
  }

  /**
   * Runs the provided code as a runnable task. If the code throws an exception,
   * it is caught, and an uncaught exception is passed to the default uncaught
   * exception handler for the current thread.
   *
   * @param code The runnable task to be executed.
   */
  public static void runTest(final Runnable code) {
    try {
      code.run();
    } catch (Exception e) {
      adam.uncaughtException(Thread.currentThread(), e);
    }
  }
}
