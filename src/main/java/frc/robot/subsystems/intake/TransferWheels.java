// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utilities.DeviceConfigurator;
import frc.lib.utilities.DeviceConstants.MotorConstants;
import frc.lib.utilities.MotorConstantsBuilder;
import frc.lib.utilities.StatedSubsystemUtils.VoltageSubsystem;
import frc.lib.utilities.StatedSubsystemUtils.VoltageSubsystemState;
import org.littletonrobotics.junction.Logger;

public class TransferWheels extends VoltageSubsystem {

  MotorConstants m_constants;

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;

  public TransferWheels() {
    m_constants = TransferWheelsConstants.motorConstants;

    m_motor = new CANSparkMax(m_constants.kID, m_constants.kMotorType);

    m_encoder = m_motor.getEncoder();
    m_controller = m_motor.getPIDController();

    DeviceConfigurator.configureSpark(m_motor, m_encoder, m_controller, m_constants);

    SmartDashboard.putBoolean("isNoteInTransfer", false);
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public double getVoltage() {
    return m_motor.getAppliedOutput();
  }

  public Command wheelsOff() {
    return setVoltageCommand(TransferWheelsConstants.kOff);
  }

  public Command wheelsToLauncher() {
    return setVoltageCommand(TransferWheelsConstants.kWheelsToLauncher);
  }

  public Command wheelsToAmper() {
    return setVoltageCommand(TransferWheelsConstants.kWheelsToAmper);
  }

  public boolean getIsNoteInTransfer() {
    return SmartDashboard.getBoolean("isNoteInTransfer", false);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Transfer Wheels Current State", m_currentState);
    Logger.recordOutput("Transfer Wheels Current Voltage", getVoltage());
  }

  public class TransferWheelsConstants {
    public static final VoltageSubsystemState kOff = new VoltageSubsystemState(0, "Off");
    public static final VoltageSubsystemState kWheelsToLauncher =
        new VoltageSubsystemState(12, "Wheels To Launcher");
    public static final VoltageSubsystemState kWheelsToAmper =
        new VoltageSubsystemState(-12, "Wheels To Amper");

    public static final MotorConstants motorConstants =
        new MotorConstantsBuilder()
            .withCurrentLimit(80)
            .withHomePosition(0)
            .withID(14)
            .withMotorType(MotorType.kBrushless)
            .withIsIdleBreak(true)
            .withInverted(false)
            .withName("Transfer Wheels Motor")
            .build();
  }
}
