package frc.robot.subsystems.LED.effects;

import frc.robot.subsystems.LED.LED.LEDEffect;
import frc.robot.subsystems.LED.LED.RGB;
import frc.robot.subsystems.LED.LED.StripSegment;
import java.util.ArrayList;

public class ListEffect extends TimedEffect implements LEDEffect {

  private ArrayList<Double> m_effectList = new ArrayList<>();

  public ListEffect(
      double effectTime, RGB color, StripSegment segment, ArrayList<Double> effectListBlueprint) {
    super(effectTime, color, segment);
    m_color = color;
    m_segment = segment;

    extendEffectList(effectListBlueprint);
  }

  private void extendEffectList(ArrayList<Double> blueprint) {
    for (int i = 0; i < blueprint.size(); i++) {
      m_effectList.add(blueprint.get(i));
    }

    while (m_effectList.size() < (m_segment.endIndex() - m_segment.startIndex())) {
      m_effectList.add(0.0);
    }
  }

  private void addListToBuffer() {
    for (int i = 0; i < m_effectList.size(); i++) {
      m_buffer.setRGB(
          i + m_segment.startIndex(),
          (int) (m_color.green() * m_effectList.get(i)),
          (int) (m_color.blue() * m_effectList.get(i)),
          (int) (m_color.red() * m_effectList.get(i)));
    }
  }

  @Override
  public void runEffect() {
    incrementTime();
    addListToBuffer();
  }

  @Override
  public void incrementFrame() {
    m_effectList.add(0, m_effectList.remove(m_effectList.size() - 1));
  }
}
