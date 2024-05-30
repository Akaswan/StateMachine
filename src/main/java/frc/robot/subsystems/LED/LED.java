// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {

  public record RGB(int red, int green, int blue) {}

  /** Creates a new LED. */
  private static LED m_instance = null;

  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  private LEDEffect m_currentEffect;

  public LED() {
    m_led = new AddressableLED(LEDConstants.kLEDPort);
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

    m_led.setLength(LEDConstants.kLEDLength);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public static LED getInstance() {
    if (m_instance == null) {
      m_instance = new LED();
    }

    return m_instance;
  }

  public void setEffect(LEDEffect effect) {
    m_currentEffect = effect;
  }

  @Override
  public void periodic() {
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


