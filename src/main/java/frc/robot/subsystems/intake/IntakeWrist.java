package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utilities.DeviceConfigurator;
import frc.lib.utilities.DeviceConstants.MotorConstants;
import frc.lib.utilities.MotorConstantsBuilder;
import frc.lib.utilities.StatedSubsystemUtils.PositionSubsystem;
import frc.lib.utilities.StatedSubsystemUtils.PositionSubsystemState;
import frc.lib.utilities.TrapezoidProfileRunner;
import org.littletonrobotics.junction.Logger;

public class IntakeWrist extends PositionSubsystem {

  private MotorConstants m_constants;

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;

  private final TrapezoidProfileRunner m_profileRunner;

  private double m_simPosition;

  public IntakeWrist() {
    super(IntakeWristConstants.kSetpointTolerance);
    m_constants = IntakeWristConstants.motorConstants;

    m_motor = new CANSparkMax(m_constants.kID, m_constants.kMotorType);

    m_encoder = m_motor.getEncoder();
    m_controller = m_motor.getPIDController();

    DeviceConfigurator.configureSpark(m_motor, m_encoder, m_controller, m_constants);

    m_simPosition = m_constants.kHomePosition;
    m_desiredPosition = m_simPosition;

    runToPosition(m_constants.kHomePosition);

    m_profileRunner =
        new TrapezoidProfileRunner(
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    m_constants.kMaxVelocity, m_constants.kMaxAcceleration)),
            (position) -> runToPosition(position),
            this::getPosition,
            IntakeWristConstants.kSetpointTolerance);
  }

  @Override
  public double getPosition() {
    return RobotBase.isReal() ? m_encoder.getPosition() : m_simPosition;
  }

  @Override
  public void runToPosition(double position) {
    m_simPosition = position;
    m_controller.setReference(position, ControlType.kPosition);
  }

  @Override
  public void runProfileToPosition(double position) {
    m_profileRunner.startProfile(position);
  }

  public Command intakeStowed() {
    return runProfileToPositionCommand(IntakeWristConstants.kStowed);
  }

  public Command intakeDown() {
    return runProfileToPositionCommand(IntakeWristConstants.kDown);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake Current State", m_currentState);
    Logger.recordOutput("Intake Desired State", m_desiredState);

    Logger.recordOutput("Intake Desired Position", m_desiredPosition);
    Logger.recordOutput("Intake Current Position", getPosition());

    Logger.recordOutput("Intake Desired Velocity", m_simPosition);

    Logger.recordOutput("Intake Is At Setpoint", getIsAtSetpoint());
  }

  public class IntakeWristConstants {
    public static final PositionSubsystemState kStowed = new PositionSubsystemState(90, "Stowed");
    public static final PositionSubsystemState kDown = new PositionSubsystemState(270, "Down");

    public static final double kSetpointTolerance = 1;

    public static final MotorConstants motorConstants =
        new MotorConstantsBuilder()
            .withCurrentLimit(80)
            .withHomePosition(kStowed.position())
            .withID(3)
            .withIsIdleBreak(true)
            .withInverted(false)
            .withKp(.1)
            .withPositionConversion(1)
            .withName("Intake Motor")
            .withMaxAcceleration(500)
            .withMaxVelocity(500)
            .build();
  }
}
