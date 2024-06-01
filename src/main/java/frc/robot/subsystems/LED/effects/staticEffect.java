package frc.robot.subsystems.LED.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.LED.LED.LEDEffect;
import frc.robot.subsystems.LED.LED.RGB;
import frc.robot.subsystems.LED.LED.StripSegment;

public class staticEffect implements LEDEffect {

    private RGB m_color;
    private AddressableLEDBuffer m_buffer;
    private StripSegment m_segment;

    public staticEffect(RGB color, AddressableLEDBuffer buffer, StripSegment segment) {
        m_color = color;
        m_buffer = buffer;
        m_segment = segment;
    }

    @Override
    public void runEffect() {
        for (int i = m_segment.startIndex(); i < m_segment.endIndex(); i++) {
            m_buffer.setRGB(
                i,
                (int) (m_color.red()),
                (int) (m_color.green()),
                (int) (m_color.blue()));
          }
    }

    @Override
    public StripSegment getSegment() {
        return m_segment;
    }
    
}
