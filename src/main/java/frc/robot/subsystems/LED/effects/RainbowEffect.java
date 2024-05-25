package frc.robot.subsystems.LED.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.LED.LED.LEDEffect;

public class RainbowEffect implements LEDEffect {

  private int m_rainbowFirstPixelHue = 0;

  @Override
  public void runEffect(AddressableLEDBuffer buffer) {
    // For every pixel
    for (var i = 0; i < buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
      // Set buffer value
      buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
}
