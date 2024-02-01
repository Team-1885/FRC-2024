package frc.robot.hardware.vendors.thirdparties;

public class WS2812 {
  public enum EAddressableLEDData{
    DESIRED_COLOR,
    ACTUAL_COLOR
  }
  public enum ELEDControlData {
    LED_STATE, // 0 for solid color, 1 for blinking
    DESIRED_COLOR,
    BLINK_SPEED;
  }
}
