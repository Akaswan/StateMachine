// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amper.AmperWheels;
import frc.robot.subsystems.amper.AmperWheels.AmperWheelsConstants;

public class AmperWheelsTrapping extends Command {
  /** Creates a new AmperWheelsTrap. */
  private AmperWheels m_amperWheels;

  private double m_operationInterval;

  private Timer m_timer = new Timer();

  public AmperWheelsTrapping(AmperWheels amperWheels, double operationInterval) {
    m_amperWheels = amperWheels;
    m_operationInterval = operationInterval;

    addRequirements(m_amperWheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.advanceIfElapsed(m_operationInterval)) {
      if (m_amperWheels.getCurrentState().equals(AmperWheelsConstants.kOff.name())) {
        m_amperWheels.setVoltage(AmperWheelsConstants.kTrapping.voltage());
        m_amperWheels.setCurrentState(AmperWheelsConstants.kTrapping.name());
      } else {
        m_amperWheels.setVoltage(AmperWheelsConstants.kOff.voltage());
        m_amperWheels.setCurrentState(AmperWheelsConstants.kOff.name());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
