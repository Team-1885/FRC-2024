package frc.robot.hardware.vendors.thirdparties.ctre;

import java.util.logging.Level;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.ADAM;
import frc.robot.hardware.vendors.thirdparties.revlib.REVLibCAN;
import lombok.Getter;

/**
 * {@linkplain} https://docs.wpilib.org/en/stable/docs/software/can-devices/third-party-devices.html
 */
public class CTRETalonSRX {
  
  // ====================================
  // DO NOT EDIT THESE PHYSICAL CONSTANTS
  // ====================================

  public static @Getter final int TIMEOUT_MS = 100;

  // =======================================
  // DO NOT EDIT THESE CONFIGURATION OPTIONS
  // =======================================

  public static @Getter final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
  public static @Getter final double NEUTRAL_DEADBAND = 0.04;
  public static @Getter final boolean ENABLE_CURRENT_LIMIT = false;
  public static @Getter final boolean ENABLE_SOFT_LIMIT = false;
  public static @Getter final boolean ENABLE_LIMIT_SWITCH = false;
  public static @Getter final int FORWARD_SOFT_LIMIT = 0;
  public static @Getter final int REVERSE_SOFT_LIMIT = 0;
  public static @Getter final boolean INVERTED = false;
  public static @Getter final boolean SENSOR_PHASE = false;
  public static @Getter final int CONTROL_FRAME_PERIOD_MS = 5;
  public static @Getter final int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
  public static @Getter final int GENERAL_STATUS_FRAME_RATE_MS = 5;
  public static @Getter final int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
  public static @Getter final int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
  public static @Getter final int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
  public static @Getter final int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;
  public static @Getter final int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;
  public static @Getter final double OPEN_LOOP_RAMP_RATE = 0.0;
  public static @Getter final double CLOSED_LOOP_RAMP_RATE = 0.0;

  private @Getter static ADAM adam = new ADAM(null);
  private static final Logger logger = Logger.getLogger(CTRETalonSRX.class.getName());

  public static void logFaults(TalonSRX CTRETalonSRX) {
    runTest(() -> {
      String separator = "// ==========";
      Faults CTRE_0xF = new Faults();
      CTRETalonSRX.getFaults(CTRE_0xF);
  
      if(CTRE_0xF.hasAnyFault()) {
        System.out.println("Faults detected on TalonSRX ID " + CTRETalonSRX.getDeviceID());
        if(CTRE_0xF.ForwardLimitSwitch) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Forward limit switch is tripped and device is trying to go forward Only trips when the device is limited");
          System.out.println(separator);
        }
        if(CTRE_0xF.ForwardSoftLimit) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Sensor is beyond forward soft limit and device is trying to go forward Only trips when the device is limited");
          System.out.println(separator);
        }
        if(CTRE_0xF.HardwareESDReset) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "???");
          System.out.println(separator);
        }
        if(CTRE_0xF.HardwareFailure) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Device detects hardware failure");
          System.out.println(separator);
        }
        if(CTRE_0xF.RemoteLossOfSignal) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Remote Sensor is no longer detected on bus");
          System.out.println(separator);
        }
        if(CTRE_0xF.ResetDuringEn) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Device was powered-on or reset while robot is enabled. Check your breakers and wiring.");
          System.out.println(separator);
        }
        if(CTRE_0xF.ReverseLimitSwitch) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Reverse limit switch is tripped and device is trying to go reverse Only trips when the device is limited");
          System.out.println(separator);
        }
        if(CTRE_0xF.ReverseSoftLimit) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Sensor is beyond reverse soft limit and device is trying to go reverse Only trips when the device is limited");
          System.out.println(separator);
        }
        if(CTRE_0xF.SensorOutOfPhase) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Device detects its sensor is out of phase");
          System.out.println(separator);
        }
        if(CTRE_0xF.SensorOverflow) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Device's sensor overflowed");
          System.out.println(separator);
        }
        if(CTRE_0xF.SupplyOverV) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Supply is well above the rated voltage of the hardware. This fault is specific to Brushless.");
          System.out.println(separator);
        }
        if(CTRE_0xF.SupplyUnstable) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Supply is rapidly fluctuating and unstable. This fault is specific to Brushless.");
          System.out.println(separator);
        }
        if(CTRE_0xF.UnderVoltage) {
          System.out.println(separator);
          logger.log(null, "// ", Level.SEVERE + "Motor Controller is under 6.5V");
          System.out.println(separator);
        }
      }
    });
  }

  /**
   * ...
   */
  public void debugClass() {
    runTest(() -> logFaults(null));
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
