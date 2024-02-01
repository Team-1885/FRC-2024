package frc.robot.network;

import edu.wpi.first.net.PortForwarder;
import lombok.Getter;

/**
 * This class provides an easy way to forward local ports to another host/port. This is useful to provide a way to access Ethernet-connected devices from a computer tethered to the roboRIO USB port.
 * Documentation: {@linkplain} https://docs.wpilib.org/en/latest/docs/networking/networking-utilities/portforwarding.html
 */
public enum EForwardableConnections {

  LIMELIGHT_CAMERA_FEED("10.18.85.11", 5800),
  LIMELIGHT_WEB_VIEW("10.18.85.11", 5801);

  private final @Getter String IPAddress;
  private final @Getter int portNumber;
  private EForwardableConnections(String IPAddress, int portNumber) {
    this.IPAddress = IPAddress;
    this.portNumber = portNumber;
  }

  public static void portForwarding(EForwardableConnections externalConnection) {
    PortForwarder.add(externalConnection.getPortNumber(), externalConnection.getIPAddress(), externalConnection.getPortNumber());
  }

  public static void dissolveExternalConnection(EForwardableConnections externalConnection) {
    // PortForwarder.remove(0);
  }
}
