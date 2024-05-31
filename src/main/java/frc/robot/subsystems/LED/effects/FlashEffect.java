package frc.robot.subsystems.LED.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.LED.LED.LEDEffect;
import frc.robot.subsystems.LED.LED.RGB;
import frc.robot.subsystems.LED.LED.StripSegment;

public class FlashEffect extends TimedEffect implements LEDEffect {

  private RGB m_color;

  public FlashEffect(RGB color, double effectTime) {
    super(effectTime);
    m_color = color;
  }

  @Override
  public void runEffect(AddressableLEDBuffer buffer, StripSegment segment) {
    double multiplier =
        percentPassed > 0.5 ? 0 : 1;
    for (int i = segment.startIndex(); i <= segment.endIndex(); i++) {
      buffer.setRGB(
          i,
          (int) (m_color.green() * multiplier),
          (int) (m_color.blue() * multiplier),
          (int) (m_color.red() * multiplier));
    }

    incrementTime();
  }
}
