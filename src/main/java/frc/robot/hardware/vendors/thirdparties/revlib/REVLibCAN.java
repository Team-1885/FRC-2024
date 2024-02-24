package frc.robot.hardware.vendors.thirdparties.revlib;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.stream.Stream;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.ADAM;
import lombok.Getter;

/**
 * {@linkplain} https://docs.wpilib.org/en/stable/docs/software/can-devices/third-party-devices.html
 */
public class REVLibCAN {

  // ====================================
  // DO NOT EDIT THESE PHYSICAL CONSTANTS
  // ====================================

  public static @Getter final double GEARBOX_RATIO = (12.0 / 40.0) * (16.0 / 38.0);
  public static @Getter final double WHEEL_DIAMETER_FT = 0.5;
  public static @Getter final double WHEEL_DIAMETER_IN = 6;
  public static @Getter final double TRACK_WIDTH_FEET = 1.8;
  public static @Getter final double WHEEL_CIRCUM_FT = WHEEL_DIAMETER_FT * Math.PI;
  public static @Getter final double NEO_POS_FACTOR = GEARBOX_RATIO * WHEEL_CIRCUM_FT;
  public static @Getter final double NEO_VEL_FACTOR = NEO_POS_FACTOR / 60.0;
  public static @Getter final double MAX_VELOCITY_RPM = 5676;
  public static @Getter final double PULSES_PER_ROTATION = 256.0;
  public static @Getter final double CURRENT_LIMIT_AMPS = 60.0;
  public static @Getter final int L_MASTER_ID = 10;
  public static @Getter final int L_FOLLOWER_ID = 9;
  public static @Getter final int R_MASTER_ID = 4;
  public static @Getter final int R_FOLLOWER_ID = 2;

  public static @Getter final int INTAKE_FEEDER_ID = 1;
  public static @Getter final int INTAKE_ROTATER_ID = 3;

  // =======================================
  // DO NOT EDIT THESE CONFIGURATION OPTIONS
  // =======================================

  public static @Getter int CAN_TIMEOUT = 100;
  public static @Getter int CONTROL_FRAME_PERIOD = 10;
  public static @Getter IdleMode IDLE_MODE = IdleMode.kCoast;
  public static @Getter int STATUS_0_PERIOD_MS = 10;
  public static @Getter int STATUS_1_PERIOD_MS = 20;
  public static @Getter int STATUS_2_PERIOD_MS = 50;
  public static @Getter double RAMP_RATE = 0.0;
  public static @Getter int SMART_CURRENT_LIMIT = 55;
  public static @Getter double SECONDARY_CURRENT_LIMIT = 40.0;
  public static @Getter final CANSparkLowLevel.MotorType MOTOR_TYPE = CANSparkLowLevel.MotorType.kBrushless;

  private @Getter static ADAM adam = new ADAM(null);
  private static final Logger logger = Logger.getLogger(REVLibCAN.class.getName());

  public static void logFaults(Stream<CANSparkMax> REVLibCAN) {
    runTest(() -> {
      REVLibCAN.forEach(motor -> {
        reportFaults(motor);
        reportStickyFaults(motor);
      });
    });
  }

  //attempt to fix logFaults error on TalonIntake, didn't work :(
  /* public static void logFaults(Stream<FalconFX> REVLibCAN) {
    runTest(() -> {
      REVLibCAN.forEach(motor -> {
        reportFaults(motor);
        reportStickyFaults(motor);
      });
    });
  } */

  public static void reportFaults(CANSparkMax REVLibCAN) {
    runTest(() -> {
      if (hasFaults(REVLibCAN)) {
        getFaultList(REVLibCAN, fault -> logger.log(Level.SEVERE,
            "Fault " + fault.name() + " detected on Spark MAX ID " + REVLibCAN.getDeviceId()));
      }
    });
  }

  public static void reportStickyFaults(CANSparkMax REVLibCAN) {
    runTest(() -> {
      if (hasFaults(REVLibCAN)) {
        getStickyFaultList(REVLibCAN, fault -> logger.log(Level.SEVERE,
            "Sticky Fault " + fault.name() + " detected on Spark MAX ID " + REVLibCAN.getDeviceId()));
      }
    });
  }

  public static List<CANSparkMax.FaultID> getFaultList(CANSparkMax REVLibCAN,
      Consumer<CANSparkMax.FaultID> ErrorConsumer) {
    List<CANSparkMax.FaultID> faults = new ArrayList<>();

    for (CANSparkMax.FaultID fault : CANSparkMax.FaultID.values()) {
      if (REVLibCAN.getFault(fault)) {
        faults.add(fault);
        ErrorConsumer.accept(fault);
      }
    }

    return faults;
  }

  public static List<CANSparkMax.FaultID> getStickyFaultList(CANSparkMax pCANSparkMax,
      Consumer<CANSparkMax.FaultID> pErrorConsumer) {
    List<CANSparkMax.FaultID> faults = new ArrayList<>();

    for (CANSparkMax.FaultID fault : CANSparkMax.FaultID.values()) {
      if (pCANSparkMax.getStickyFault(fault)) {
        faults.add(fault);
        pErrorConsumer.accept(fault);
      }
    }

    return faults;
  }

  public static List<CANSparkMax.FaultID> getFaultList(CANSparkMax REVLibCAN) {
    return getFaultList(REVLibCAN, pS -> {
    });
  }

  public static List<CANSparkMax.FaultID> getStickyFaultList(CANSparkMax REVLibCAN) {
    return getStickyFaultList(REVLibCAN, pS -> {
    });
  }

  public static boolean hasFaults(CANSparkMax REVLibCAN) {
    // Faults are stored in a 16-bit variable. Each bit represents a fault when its
    // value is 1.
    return REVLibCAN.getFaults() != 0;
  }

  public static boolean hasStickyFaults(CANSparkMax REVLibCAN) {
    return REVLibCAN.getStickyFaults() != 0;
  }

  /**
   * ...
   */
  public void debugClass() {
    // ...
  }

  /**
   * Runs the provided code as a runnable task. If the code throws an exception, it is caught, and an uncaught exception is passed to the default uncaught exception handler for the current thread.
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
