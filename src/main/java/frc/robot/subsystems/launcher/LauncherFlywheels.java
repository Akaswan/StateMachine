package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utilities.DeviceConfigurator;
import frc.lib.utilities.DeviceConstants.MotorConstants;
import frc.lib.utilities.MotorConstantsBuilder;
import frc.lib.utilities.StatedSubsystemUtils.VelocitySubsystem;
import frc.lib.utilities.StatedSubsystemUtils.VelocitySubsystemState;
import frc.robot.subsystems.intake.IntakeWrist.IntakeWristConstants;
import org.littletonrobotics.junction.Logger;

public class LauncherFlywheels extends VelocitySubsystem {

  private MotorConstants m_topConstants;
  private MotorConstants m_bottomConstants;

  private CANSparkFlex m_topMotor;
  private RelativeEncoder m_topEncoder;
  private SparkPIDController m_topController;

  private CANSparkFlex m_bottomMotor;
  private RelativeEncoder m_bottomEncoder;
  private SparkPIDController m_bottomController;

  private double m_topDesiredVelocity;
  private double m_bottomDesiredVelocity;

  public LauncherFlywheels() {
    m_topConstants = LauncherFlywheelsConstants.kTopMotorConstants;
    m_bottomConstants = LauncherFlywheelsConstants.kBottomMotorConstants;

    m_topMotor = new CANSparkFlex(m_topConstants.kID, MotorType.kBrushless);
    m_topEncoder = m_topMotor.getEncoder();
    m_topController = m_topMotor.getPIDController();

    m_bottomMotor = new CANSparkFlex(m_bottomConstants.kID, MotorType.kBrushless);
    m_bottomEncoder = m_bottomMotor.getEncoder();
    m_bottomController = m_bottomMotor.getPIDController();

    DeviceConfigurator.configureSpark(m_topMotor, m_topEncoder, m_topController, m_topConstants);
    DeviceConfigurator.configureSpark(
        m_bottomMotor, m_bottomEncoder, m_bottomController, m_bottomConstants);
  }

  @Override
  public boolean getIsAtSetpoint() {
    return Math.abs(getVelocity()[0] - m_topDesiredVelocity)
            < IntakeWristConstants.kSetpointTolerance
        && Math.abs(getVelocity()[1] - m_bottomDesiredVelocity)
            < IntakeWristConstants.kSetpointTolerance;
  }

  @Override
  public double[] getVelocity() {
    return new double[] {m_topEncoder.getVelocity(), m_bottomEncoder.getVelocity()};
  }

  @Override
  public void runToVelocity(double... velocities) {
    m_topDesiredVelocity = velocities[0];
    m_bottomDesiredVelocity = velocities[1];

    m_topController.setReference(velocities[0], ControlType.kVelocity);
    m_bottomController.setReference(velocities[1], ControlType.kVelocity);
  }

  public Command flywheelsOff() {
    return runToVelocityCommand(LauncherFlywheelsConstants.kOff);
  }

  public Command flywheelsRunning() {
    return runToVelocityCommand(LauncherFlywheelsConstants.kRunning);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Launcher Flywheels Current State", m_currentState);
    Logger.recordOutput("Launcher Flywheels Desired State", m_desiredState);

    Logger.recordOutput("Launcher Flywheels Current Velocity", getVelocity());
    Logger.recordOutput(
        "Launcher Flywheels Desired Velocity",
        new double[] {m_topDesiredVelocity, m_bottomDesiredVelocity});
  }

  public final class LauncherFlywheelsConstants {
    public static final double kSetpointTolerance = 1;

    public static final VelocitySubsystemState kOff = new VelocitySubsystemState("Off", 0, 0);
    public static final VelocitySubsystemState kRunning =
        new VelocitySubsystemState("Running", 6000, 4000);

    public static final MotorConstants kTopMotorConstants =
        new MotorConstantsBuilder()
            .withCurrentLimit(80)
            .withHomePosition(0)
            .withID(18)
            .withMotorType(MotorType.kBrushless)
            .withIsIdleBreak(false)
            .withInverted(false)
            .withName("Top Launcher Motor")
            .withKff(.1214)
            .withKp(.001)
            .build();

    public static final MotorConstants kBottomMotorConstants =
        new MotorConstantsBuilder()
            .withCurrentLimit(80)
            .withHomePosition(0)
            .withID(15)
            .withMotorType(MotorType.kBrushless)
            .withIsIdleBreak(false)
            .withInverted(false)
            .withName("Top Launcher Motor")
            .withKff(.1214)
            .withKp(.001)
            .build();
  }
}
