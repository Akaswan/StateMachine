// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.amper;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utilities.DeviceConfigurator;
import frc.lib.utilities.DeviceConstants.MotorConstants;
import frc.lib.utilities.MotorConstantsBuilder;
import frc.lib.utilities.StatedSubsystemUtils.VoltageSubsystem;
import frc.lib.utilities.StatedSubsystemUtils.VoltageSubsystemState;
import frc.robot.commands.amper.AmperWheelsTrapping;
import org.littletonrobotics.junction.Logger;

public class AmperWheels extends VoltageSubsystem {

  MotorConstants m_constants;

  private TalonFX m_motor;
  private TalonFXSimState m_motorSim;

  private final DCMotorSim m_motorSimModel;

  public AmperWheels() {
    m_constants = AmperWheelsConstants.motorConstants;

    m_motor = new TalonFX(m_constants.kID);

    DeviceConfigurator.configureTalonFX(m_motor, m_constants);

    m_motorSim = m_motor.getSimState();

    m_motorSimModel = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.001);
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setControl(new VoltageOut(voltage));
  }

  @Override
  public double getVoltage() {
    return m_motor.getMotorVoltage().getValue();
  }

  public Command wheelsOff() {
    return setVoltageCommand(AmperWheelsConstants.kOff);
  }

  public Command wheelsAmping() {
    return setVoltageCommand(AmperWheelsConstants.kAmping);
  }

  public Command wheelsTrapping() {
    return new AmperWheelsTrapping(this, .2);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Amper Wheels Current State", m_currentState);
    Logger.recordOutput("Amper Wheels Current Voltage", getVoltage());
  }

  @Override
  public void simulationPeriodic() {
    m_motorSimModel.setInputVoltage(m_motorSim.getMotorVoltage());
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    m_motorSim.setRawRotorPosition(m_motorSimModel.getAngularPositionRotations());
    m_motorSim.setRotorVelocity(
        Units.radiansToRotations(m_motorSimModel.getAngularVelocityRadPerSec()));
  }

  public class AmperWheelsConstants {
    public static final VoltageSubsystemState kOff = new VoltageSubsystemState(0, "Off");
    public static final VoltageSubsystemState kAmping = new VoltageSubsystemState(12, "Amping");
    public static final VoltageSubsystemState kTrapping = new VoltageSubsystemState(12, "Trapping");

    public static final MotorConstants motorConstants =
        new MotorConstantsBuilder()
            .withCurrentLimit(80)
            .withHomePosition(0)
            .withID(6)
            .withIsIdleBreak(true)
            .withInverted(false)
            .withName("Amper Wheels Motor")
            .build();
  }
}
