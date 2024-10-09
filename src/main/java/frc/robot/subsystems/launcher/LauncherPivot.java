package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utilities.DeviceConfigurator;
import frc.lib.utilities.DeviceConstants.MotorConstants;
import frc.lib.utilities.MotorConstantsBuilder;
import frc.lib.utilities.StatedSubsystemUtils.PositionSubsystem;
import frc.lib.utilities.StatedSubsystemUtils.PositionSubsystemState;
import org.littletonrobotics.junction.Logger;

public class LauncherPivot extends PositionSubsystem {

  private MotorConstants m_constants;

  private TalonFX m_motor;
  private TalonFXSimState m_motorSim;

  private final DCMotorSim m_motorSimModel;

  public LauncherPivot() {
    super(LauncherPivotConstants.kSetpointTolerance);
    m_constants = LauncherPivotConstants.motorConstants;
    m_motor = new TalonFX(m_constants.kID);

    DeviceConfigurator.configureTalonFX(m_motor, m_constants);

    m_motorSimModel = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.001);

    m_motorSim = m_motor.getSimState();

    runToPosition(m_constants.kHomePosition);
  }

  @Override
  public double getPosition() {
    return m_motor.getPosition().getValue();
  }

  @Override
  public void runToPosition(double position) {
    m_motor.setControl(new PositionVoltage(position));
  }

  @Override
  public void runProfileToPosition(double position) {
    m_motor.setControl(new MotionMagicVoltage(position));
  }

  public Command pivotDown() {
    return runProfileToPositionCommand(LauncherPivotConstants.kDown);
  }

  public Command pivotUp() {
    return runProfileToPositionCommand(LauncherPivotConstants.kUp);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Pivot Current State", m_currentState);
    Logger.recordOutput("Pivot Desired State", m_desiredState);

    Logger.recordOutput("Pivot Desired Position", m_desiredPosition);
    Logger.recordOutput("Pivot Current Position", getPosition());

    Logger.recordOutput("Pivot Is At Setpoint", getIsAtSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    m_motorSimModel.setInputVoltage(m_motorSim.getMotorVoltage());
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    m_motorSim.setRawRotorPosition(m_motorSimModel.getAngularPositionRotations());
    m_motorSim.setRotorVelocity(
        Units.radiansToRotations(m_motorSimModel.getAngularVelocityRadPerSec()));
  }

  public final class LauncherPivotConstants {
    public static final double kSetpointTolerance = 1;

    public static final PositionSubsystemState kDown = new PositionSubsystemState(0, "Down");
    public static final PositionSubsystemState kUp = new PositionSubsystemState(1, "Up");

    public static final MotorConstants motorConstants =
        new MotorConstantsBuilder()
            .withCurrentLimit(80)
            .withHomePosition(0)
            .withID(4)
            .withIsIdleBreak(true)
            .withInverted(false)
            .withKp(11.5)
            .withPositionConversion(50)
            .withName("Amp Elevator Motor")
            .withMaxAcceleration(200)
            .withMaxVelocity(100)
            .build();
  }
}
