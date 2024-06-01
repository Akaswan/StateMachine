package frc.robot.subsystems.LED.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED.RGB;
import frc.robot.subsystems.LED.LED.StripSegment;

public abstract class TimedEffect {
  protected double m_effectTime;
  protected double m_timePassed = 0;
  protected double m_percentPassed = 0;

  protected RGB m_color;
  protected AddressableLEDBuffer m_buffer = LED.getInstance().getBuffer();
  protected StripSegment m_segment;

  public TimedEffect(double effectTime, RGB color, StripSegment segment) {
    m_effectTime = effectTime;
    m_color = color;
    m_segment = segment;
  }

  public void incrementTime() {
    if ((m_timePassed + Constants.kdt) > m_effectTime) {
      m_timePassed = (m_timePassed + Constants.kdt) % m_effectTime;
      incrementFrame();
    } else {
      m_timePassed += Constants.kdt;
    }
    m_percentPassed = m_timePassed / m_effectTime;
  }

  public void incrementFrame() {}

  public void setEffectToBuffer(double multiplier) {
    for (int i = m_segment.startIndex(); i < m_segment.endIndex(); i++) {
      m_buffer.setRGB(
          i,
          (int) (m_color.green() * multiplier),
          (int) (m_color.blue() * multiplier),
          (int) (m_color.red() * multiplier));
    }
  }

  public StripSegment getSegment() {
    return m_segment;
  }
}
