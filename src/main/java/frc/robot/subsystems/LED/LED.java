// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import java.util.HashMap;
import java.util.Map;

public class LED extends SubsystemBase {

  public record RGB(int red, int green, int blue) {}

  public record StripSegment(String name, int startIndex, int endIndex)
      implements Comparable<StripSegment> {
    @Override
    public int compareTo(StripSegment other) {
      return Integer.compare(this.startIndex, other.startIndex);
    }
  }

  /** Creates a new LED. */
  private static LED m_instance = null;

  private StripSegment[] m_stripSegments;

  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  private Map<StripSegment, LEDEffect> m_effectMap = new HashMap<>();

  public LED(StripSegment... stripSegments) {
    m_stripSegments = stripSegments;

    m_led = new AddressableLED(LEDConstants.kLEDPort);
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

    for (int i = 0; i < m_stripSegments.length; i++) {
      m_effectMap.put(m_stripSegments[i], null);
    }

    m_led.setLength(LEDConstants.kLEDLength);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public static LED getInstance() {
    if (m_instance == null) {
      m_instance = new LED(LEDConstants.kArmLeftSegment, LEDConstants.kArmRightSegment);
    }

    return m_instance;
  }

  public void setEffect(LEDEffect effect, StripSegment segment) {
    m_effectMap.replace(segment, effect);
  }

  @Override
  public void periodic() {
    for (Map.Entry<StripSegment, LEDEffect> entry : m_effectMap.entrySet()) {
      if (entry.getValue() != null) {
        entry.getValue().runEffect(m_ledBuffer, entry.getKey());
      } else {
        for (int i = entry.getKey().startIndex(); i <= entry.getKey().endIndex(); i++) {
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
      }
    }

    m_led.setData(m_ledBuffer);
  }

  public interface LEDEffect {
    void runEffect(AddressableLEDBuffer buffer, StripSegment segment);
  }
}
