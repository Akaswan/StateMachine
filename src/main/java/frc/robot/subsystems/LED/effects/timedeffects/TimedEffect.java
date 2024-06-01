package frc.robot.subsystems.LED.effects.timedeffects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED.RGB;
import frc.robot.subsystems.LED.LED.StripSegment;

public abstract class TimedEffect {
  protected double m_effectTime;
  protected double m_percentPassed = 0;

  protected RGB m_color;
  protected AddressableLEDBuffer m_buffer = LED.getInstance().getBuffer();
  protected StripSegment m_segment;

  protected Timer m_timer = new Timer();

  public TimedEffect(double effectTime, RGB color, StripSegment segment) {
    m_effectTime = effectTime;
    m_color = color;
    m_segment = segment;

    m_timer.start();
  }

  public void incrementTime() {
    m_percentPassed = m_timer.get() / m_effectTime;

    if (m_timer.advanceIfElapsed(m_effectTime)) {
      incrementFrame();
      m_timer.restart();
    }
  }

  public void incrementFrame() {}

  public void setEffectToBuffer(double multiplier) {
    for (int i = m_segment.startIndex(); i < m_segment.endIndex(); i++) {
      m_buffer.setRGB(
          i,
          (int) (m_color.red() * multiplier),
          (int) (m_color.green() * multiplier),
          (int) (m_color.blue() * multiplier));
    }
  }

  public StripSegment getSegment() {
    return m_segment;
  }
}
