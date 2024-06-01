package frc.robot.subsystems.LED.effects.timedeffects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED.LEDEffect;
import frc.robot.subsystems.LED.LED.StripSegment;

public class RainbowEffect extends TimedEffect implements LEDEffect {

  private double m_hueIncrement = 1;

  private StripSegment m_segment;

  public RainbowEffect(double effectTime, StripSegment segment) {
    super(effectTime, null, segment);

    m_segment = segment;

    if (m_effectTime < Constants.kdt) m_hueIncrement = Constants.kdt / m_effectTime;
  }

  private int m_rainbowFirstPixelHue = 0;

  private AddressableLEDBuffer m_buffer = LED.getInstance().getBuffer();

  @Override
  public void runEffect() {
    for (int i = m_segment.startIndex(); i < m_segment.endIndex(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_buffer.getLength())) % 180;
      m_buffer.setHSV(i, hue, 255, 128);
    }

    incrementTime();
  }

  @Override
  public void incrementFrame() {
    m_rainbowFirstPixelHue += m_hueIncrement;
    m_rainbowFirstPixelHue %= 180;
  }
}
