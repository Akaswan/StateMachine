package frc.robot.subsystems.LED.effects;

import frc.robot.Constants;

public abstract class TimedEffect {
  protected double m_effectTime = 0;
  protected double m_timePassed = 0;
  protected double m_percentPassed = 0;

  public TimedEffect(double effectTime) {
    m_effectTime = effectTime;
  }

  public void incrementTime() {
    if (m_timePassed += Constants.kdt > m_effectTime) {
      m_timePassed = (m_timePassed + Constants.kdt) % m_effectTime;
      incrementFrame();
    } else {
      m_timePassed += Constants.kdt
    }
    m_percentPassed = m_timePassed / m_effectTime;
  }

  public void incrementFrame() {}
}
