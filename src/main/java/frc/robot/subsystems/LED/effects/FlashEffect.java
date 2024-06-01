package frc.robot.subsystems.LED.effects;

import frc.robot.subsystems.LED.LED.LEDEffect;
import frc.robot.subsystems.LED.LED.RGB;
import frc.robot.subsystems.LED.LED.StripSegment;

public class FlashEffect extends TimedEffect implements LEDEffect {

  public FlashEffect(RGB color, double effectTime, StripSegment segment) {
    super(effectTime, color, segment);
    m_color = color;

    m_segment = segment;
  }

  @Override
  public void runEffect() {
    double multiplier = m_percentPassed > 0.5 ? 0 : 1;
    setEffectToBuffer(multiplier);
    incrementTime();
  }
}
