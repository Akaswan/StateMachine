// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  /** Creates a new LED. */

  public static AddressableLED m_led;
  public static AddressableLEDBuffer m_ledBuffer;

  private static LEDEffect m_currentEffect;

  private static double m_effectTimeRun = -1;

  public LED() {
    m_led = new AddressableLED(LEDConstants.kLEDPort);
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

    m_led.setLength(LEDConstants.kLEDLength);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public static void setEffect(LEDEffect effect) {
    m_currentEffect = effect;
  }

  public static void setEffect(LEDEffect effect, double effectRunTime) {
    m_currentEffect = effect;
    m_effectTimeRun = effectRunTime;
  }

  @Override
  public void periodic() {

    if (m_effectTimeRun != -1) {
      if (m_effectTimeRun - Constants.kdt <= 0) {
        m_effectTimeRun = -1;
      } else {
        m_effectTimeRun -= Constants.kdt;
      }
    }

    if (m_currentEffect != null) {
      m_currentEffect.runEffect(m_ledBuffer);
    } else {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 0, 0);
      } 
    }

    m_led.setData(m_ledBuffer);
  }

  public interface LEDEffect {
    void runEffect(AddressableLEDBuffer buffer);
  }
}
