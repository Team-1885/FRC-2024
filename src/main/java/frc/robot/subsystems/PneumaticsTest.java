/* Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project. */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsTest extends SubsystemBase {

// Solenoid corresponds to a single solenoid.
// In this case, it's connected to channel 0 of a PH with the default CAN ID.
final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

final Solenoid exampleSingle = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

// Compressor connected to a PH with a default CAN ID (1)
final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

/** Creates a new PneumaticsTest. 
* @return */
public double PneumaticsTest() {
 
  getCurrent();
  isCompressorActive();
  isPressureSwitchConnected();

  // Disable closed-loop mode on the compressor.
  m_compressor.disable();
  /* Enable closed-loop mode based on the digital pressure switch connected to the PCM/PH.
  The switch is open when the pressure is over ~120 PSI. */
  m_compressor.enableDigital();

  // Enable closed-loop mode based on the analog pressure sensor connected to the PH.
  // The compressor will run while the pressure reported by the sensor is in the
  // specified range ([70 PSI, 120 PSI] in this example).
  // Analog mode exists only on the PH! On the PCM, this enables digital control.
  m_compressor.enableAnalog(70, 120);

  // Enable closed-loop mode based on both the digital pressure switch AND the analog
  // pressure sensor connected to the PH.
  // The compressor will run while the pressure reported by the analog sensor is in the
  // specified range ([70 PSI, 120 PSI] in this example) AND the digital switch reports
  // that the system is not full.
  // Hybrid mode exists only on the PH! On the PCM, this enables digital control.
  m_compressor.enableHybrid(70, 120);

  //(I skipped a piece of code because there was a duplicate)

  /* Get the pressure (in PSI) from the analog sensor connected to the PH.
  This function is supported only on the PH!
  On a PCM, this function will return 0. */
  return m_compressor.getPressure();

  // External analog pressure sensor
  // product-specific voltage->pressure conversion, see product manual
  // in this case, 250(V/5)-25
  // the scale parameter in the AnalogPotentiometer constructor is scaled from 1 instead of 5,
  // so if r is the raw AnalogPotentiometer output, the pressure is 250r-25
  // final double kScale = 250; (may have to uncomment)
  // final double kOffset = -25; (may have to uncomment)
  // final AnalogPotentiometer m_pressureTransducer = (may have to uncomment)
  // new AnalogPotentiometer(/* the AnalogIn port*/ 2, kScale, kOffset); (may have to uncomment)

  // Get the pressure (in PSI) from an analog pressure sensor connected to the RIO.
  // return m_pressureTransducer.get(); (may have to uncomment)

}

public double getCurrent(){
// Get compressor current draw. Method used on line 28
return m_compressor.getCurrent();
}

public boolean isCompressorActive() {
// Get whether the compressor is active. Method is used on line 29
return m_compressor.isEnabled();
}

public boolean isPressureSwitchConnected() {
/* Get the digital pressure switch connected to the PCM/PH.
The switch is open when the pressure is over ~120 PSI. 
Method is used on line 30 */
return m_compressor.getPressureSwitchValue();
}
}
