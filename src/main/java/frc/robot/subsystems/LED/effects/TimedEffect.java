package frc.robot.subsystems.LED.effects;

import frc.robot.Constants;

public abstract class TimedEffect {
    protected double m_frameTime;
    protected double m_timePassed = 0;

    public TimedEffect(double frameTime) {
        m_frameTime = frameTime;
    }

    public void incrementTime() {
        m_timePassed = (m_timePassed + Constants.kdt) % m_frameTime;
    }
}