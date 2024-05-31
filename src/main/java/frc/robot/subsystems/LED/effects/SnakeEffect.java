package frc.robot.subsystems.LED.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.LED.LED.LEDEffect;
import frc.robot.subsystems.LED.LED.RGB;
import frc.robot.subsystems.LED.LED.StripSegment;

public class SnakeEffect extends TimedEffect implements LEDEffect {

  private RGB m_color;

  private ArrayList<Double> m_effectList = Arrays.asList(
    1.0, 1.0, 1.0
  );

  public SnakeEffect(RGB color, double effectTime) {
    super(effectTime);
    m_color = color;
  }

  @Override
  public void runEffect(AddressableLEDBuffer buffer, StripSegment segment) {
    incrementTime();
  }

  @Override
  public void incrementFrame() {




    m_effectList.add(0, list.remove(list.size() - 1));
  }
}
