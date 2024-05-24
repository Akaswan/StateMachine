package frc.robot.subsystems.LED.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.utilities.RGB;
import frc.robot.subsystems.LED.LED.LEDEffect;

public class BreatheEffect extends TimedEffect implements LEDEffect{

    private RGB m_color;

    public BreatheEffect(RGB color, double frameTime) {
        super(frameTime);
        m_color = color;
    }

    @Override
    public void runEffect(AddressableLEDBuffer buffer) {
        double percentPassed = m_timePassed / m_frameTime;
        double multiplier = percentPassed > 0.5 ? (percentPassed * 2) - 1 : (2 - (percentPassed * 2)) - 1;
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, (int) (m_color.red * multiplier), (int) (m_color.green * multiplier), (int) (m_color.blue * multiplier));
        }

        incrementTime();
    }
    
}
