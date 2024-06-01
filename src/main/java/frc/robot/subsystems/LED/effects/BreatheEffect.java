package frc.robot.subsystems.LED.effects;

import frc.robot.subsystems.LED.LED.LEDEffect;
import frc.robot.subsystems.LED.LED.RGB;
import frc.robot.subsystems.LED.LED.StripSegment;

public class BreatheEffect extends TimedEffect implements LEDEffect {

  public BreatheEffect(RGB color, double effectTime, StripSegment segment) {
    super(effectTime, color, segment);
    m_color = color;

    m_segment = segment;
  }

  @Override
  public void runEffect() {
    double multiplier =
        m_percentPassed > 0.5 ? (m_percentPassed * 2) - 1 : (2 - (m_percentPassed * 2)) - 1;
    setEffectToBuffer(multiplier);

    incrementTime();
  }

  @Override
  public StripSegment getSegment() {
    return m_segment;
  }
}
