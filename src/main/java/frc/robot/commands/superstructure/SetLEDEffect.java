// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED.LEDEffect;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLEDEffect extends Command {

  private LEDEffect m_effect;
  private double m_timePassed = 0.0;
  private double m_runTime;
  private LEDEffect m_queuedEffect = null;

  private LED m_led = LED.getInstance();

  /**
   *
   *
   * <h3>SetLEDEffect</h3>
   *
   * Sets the effect of the LED class then ends instantly
   *
   * @param effect
   */
  public SetLEDEffect(LEDEffect effect) {
    m_effect = effect;

    addRequirements(RobotContainer.m_LED);
  }

  /**
   *
   *
   * <h3>SetLEDEffect</h3>
   *
   * Sets the effect of the LED class then ends instantly
   *
   * <p>When using this in a command composition create a <code>new ScheduleCommand()</code> with
   * this command as its parameter
   *
   * @param effect The LEDEffect to be applied immediately.
   * @param runTime The duration (in seconds) for which the effect should run.
   * @param queuedEffect The LEDEffect to be applied after the current effect finishes.
   */
  public SetLEDEffect(LEDEffect effect, double runTime, LEDEffect queuedEffect) {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.setEffect(m_effect);
  }

  @Override
  public void execute() {
    m_timePassed += Constants.kdt;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_queuedEffect != null && !interrupted) {
      m_led.setEffect(m_queuedEffect);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timePassed >= m_runTime;
  }
}
