// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED.LEDEffect;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLEDEffect extends InstantCommand {

  LEDEffect m_effect;

  public SetLEDEffect(LEDEffect effect) {
    m_effect = effect;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LED.setEffect(m_effect);
  }
}
