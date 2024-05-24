package frc.robot.subsystems.LED.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.utilities.RGB;
import frc.robot.Constants;
import frc.robot.subsystems.LED.LED.LEDEffect;

public class BreatheEffect implements LEDEffect{

    private RGB m_color;
    private double m_frameTime;
    private double m_timePassed = 0;

    public BreatheEffect(RGB color, double frameTime) {
        m_color = color;
        m_frameTime = frameTime;
    }

    @Override
    public void runEffect(AddressableLEDBuffer buffer) {
        double percentPassed = m_timePassed / m_frameTime;
        double multiplier = percentPassed > 0.5 ? (percentPassed * 2) - 1 : (2 - (percentPassed * 2)) - 1;
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, (int) (m_color.red * multiplier), (int) (m_color.green * multiplier), (int) (m_color.blue * multiplier));
        }

        m_timePassed = (m_timePassed + Constants.kdt) % m_frameTime;
    }
    
}
