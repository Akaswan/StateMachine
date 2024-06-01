package frc.robot.subsystems.LED.effects.timedeffects;

import frc.robot.subsystems.LED.LED.LEDEffect;
import frc.robot.subsystems.LED.LED.RGB;
import frc.robot.subsystems.LED.LED.StripSegment;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public class ListEffect extends TimedEffect implements LEDEffect {

  private ArrayList<Double> m_effectList = new ArrayList<>();

  boolean m_rubberBand = false;
  boolean m_isRunningForward = true;
  int m_effectStartIndex = 0;
  int m_effectEndIndex = 0;
  double m_startWait = 0;
  double m_endWait = 0;
  Timer m_rubberBandTimer;

  public ListEffect(
      double effectTime, RGB color, StripSegment segment, ArrayList<Double> effectListBlueprint) {
    super(effectTime, color, segment);
    m_color = color;
    m_segment = segment;

    extendEffectList(effectListBlueprint);
  }

  public ListEffect(
      double effectTime,
      RGB color,
      StripSegment segment,
      ArrayList<Double> effectListBlueprint,
      boolean repeat) {
    this(effectTime, color, segment, effectListBlueprint);

    m_effectList = new ArrayList<>();
    repeatEffectList(effectListBlueprint);
  }

  public ListEffect(
      double effectTime,
      RGB color,
      StripSegment segment,
      ArrayList<Double> effectListBlueprint,
      boolean rubberBand,
      double startWait,
      double endWait) {
    this(effectTime, color, segment, effectListBlueprint);

    m_rubberBand = rubberBand;

    m_startWait = startWait;
    m_endWait = endWait;

    m_effectEndIndex = effectListBlueprint.size() - 1;

    m_rubberBandTimer = new Timer();
  }

  private void repeatEffectList(ArrayList<Double> blueprint) {
    int i = 0;

    while (m_effectList.size() < (m_segment.endIndex() - m_segment.startIndex())) {
      m_effectList.add(blueprint.get(i));
      i = (i + 1) % (blueprint.size());
    }
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
          (int) (m_color.red() * m_effectList.get(i)),
          (int) (m_color.green() * m_effectList.get(i)),
          (int) (m_color.blue() * m_effectList.get(i)));
    }
  }

  @Override
  public void runEffect() {
    incrementTime();
    addListToBuffer();
  }

  @Override
  public void incrementFrame() {
    if (m_rubberBand) {
      if (m_isRunningForward) {
        if (m_effectEndIndex == m_effectList.size() - 1) {
          m_rubberBandTimer.start();
          if (m_rubberBandTimer.hasElapsed(m_endWait)) {
            m_isRunningForward = false;
            m_effectList.add(m_effectList.size() - 1, m_effectList.remove(0));
            m_effectEndIndex -= 1;
            m_effectStartIndex -= 1;
            m_rubberBandTimer.reset();
            m_rubberBandTimer.stop();
          }
        } else {
          m_effectList.add(0, m_effectList.remove(m_effectList.size() - 1));
          m_effectEndIndex += 1;
          m_effectStartIndex += 1;
        }
      } else {
        if (m_effectStartIndex == 0) {
          m_rubberBandTimer.start();
          if (m_rubberBandTimer.hasElapsed(m_startWait)) {
            m_isRunningForward = true;
            m_effectList.add(0, m_effectList.remove(m_effectList.size() - 1));
            m_effectEndIndex += 1;
            m_effectStartIndex += 1;
            m_rubberBandTimer.reset();
            m_rubberBandTimer.stop();
          }
        } else {
          m_effectEndIndex -= 1;
          m_effectStartIndex -= 1;
          m_effectList.add(m_effectList.size() - 1, m_effectList.remove(0));
        }
      }
    } else {
      m_effectList.add(0, m_effectList.remove(m_effectList.size() - 1));
    }
  }
}
