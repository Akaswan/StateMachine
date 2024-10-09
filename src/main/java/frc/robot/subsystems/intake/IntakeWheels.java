// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utilities.DeviceConfigurator;
import frc.lib.utilities.DeviceConstants.MotorConstants;
import frc.lib.utilities.MotorConstantsBuilder;
import frc.lib.utilities.StatedSubsystemUtils.VoltageSubsystem;
import frc.lib.utilities.StatedSubsystemUtils.VoltageSubsystemState;
import org.littletonrobotics.junction.Logger;

public class IntakeWheels extends VoltageSubsystem {

  private final MotorConstants m_constants;

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;

  public IntakeWheels() {
    m_constants = IntakeWheelsConstants.motorConstants;

    m_motor = new CANSparkMax(m_constants.kID, m_constants.kMotorType);

    m_encoder = m_motor.getEncoder();
    m_controller = m_motor.getPIDController();

    DeviceConfigurator.configureSpark(m_motor, m_encoder, m_controller, m_constants);
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
    return setVoltageCommand(IntakeWheelsConstants.kOff);
  }

  public Command wheelsIntaking() {
    return setVoltageCommand(IntakeWheelsConstants.kIntaking);
  }

  public Command wheelsOuttaking() {
    return setVoltageCommand(IntakeWheelsConstants.kOuttaking);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake Wheels Current State", m_currentState);
    Logger.recordOutput("Intake Wheels Current Voltage", getVoltage());
  }

  public class IntakeWheelsConstants {

    public static final VoltageSubsystemState kOff = new VoltageSubsystemState(0, "Off");
    public static final VoltageSubsystemState kIntaking = new VoltageSubsystemState(12, "Intaking");
    public static final VoltageSubsystemState kOuttaking =
        new VoltageSubsystemState(-12, "Outtaking");

    public static final MotorConstants motorConstants =
        new MotorConstantsBuilder()
            .withCurrentLimit(80)
            .withHomePosition(0)
            .withID(13)
            .withMotorType(MotorType.kBrushless)
            .withIsIdleBreak(true)
            .withInverted(false)
            .withName("Amper Wheels Motor")
            .build();
  }
}
