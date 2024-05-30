package frc.robot.subsystems.LED.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.LED.LED.LEDEffect;
import frc.robot.subsystems.LED.LED.RGB;

public class BreatheEffect extends TimedEffect implements LEDEffect {

  private RGB m_color;

  public BreatheEffect(RGB color, double frameTime) {
    super(frameTime);
    m_color = color;
  }

  @Override
  public void runEffect(AddressableLEDBuffer buffer) {
    double percentPassed = m_timePassed / m_frameTime;
    double multiplier =
        percentPassed > 0.5 ? (percentPassed * 2) - 1 : (2 - (percentPassed * 2)) - 1;
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(
          i,
          (int) (m_color.green() * multiplier),
          (int) (m_color.blue() * multiplier),
          (int) (m_color.red() * multiplier));
    }

    incrementTime();
  }
}
