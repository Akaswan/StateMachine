package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.templates.builders.MotorConstantsBuilder;
import frc.lib.templates.builders.PositionSubsystemConstantsBuilder;
import frc.lib.templates.subsystems.PositionSubsystem;
import frc.lib.templates.subsystems.SubsystemConstants.ManualControlMode;
import frc.lib.templates.subsystems.SubsystemConstants.MotorConstants;
import frc.lib.templates.subsystems.SubsystemConstants.MotorControllerType;
import frc.lib.templates.subsystems.SubsystemConstants.PositionSubsystemConstants;

public class Arm extends PositionSubsystem {

  private static Arm m_instance = null;

  public Arm(PositionSubsystemConstants constants) {
    super(constants);
  }

  public static Arm getInstance() {
    if (m_instance == null) {
      m_instance = new Arm(ArmConstants.kArmConstants);
    }

    return m_instance;
  }

  @Override
  public void subsystemPeriodic() {}

  @Override
  public void outputTelemetry() {}

  public enum ArmState implements SubsystemState {
    DOWN(0),
    UP(45);

    private TrapezoidProfile.State state = new TrapezoidProfile.State(0, 0);

    private ArmState(double position) {
      state.position = position;
    }

    private ArmState() {}

    @Override
    public String getName() {
      return toString();
    }

    @Override
    public State getSetpoint() {
      return state;
    }

    @Override
    public double getPosition() {
      return state.position;
    }

    @Override
    public double getVelocity() {
      return state.velocity;
    }

    @Override
    public void setSetpoint(State state) {
      this.state = state;
    }

    @Override
    public void setPosition(double position) {
      state.position = position;
    }

    @Override
    public void setVelocity(double velocity) {
      state.velocity = velocity;
    }

    @Override
    public double getVoltage() {
      throw new UnsupportedOperationException(
          "'getVoltage()' is not to be used in a position subsystem");
    }

    @Override
    public void setVoltage(double voltage) {
      throw new UnsupportedOperationException(
          "'setVoltage()' is not to be used in a position subsystem");
    }
  }

  public class ArmConstants {
    public static final MotorConstants kArmRight =
        new MotorConstantsBuilder()
            .withName("Arm Right")
            .withMotorControllerType(MotorControllerType.SPARK_MAX)
            .withID(8)
            .withIdleMode(IdleMode.kBrake)
            .withMotorType(MotorType.kBrushless)
            .withCurrentLimit(80)
            .withInverted(false)
            .withHomePosition(0.0)
            .withPositionConversion(1)
            .withVelocityConversion(1 / 60)
            .withKp(0.1)
            .build();

    public static final MotorConstants kArmLeft =
        new MotorConstantsBuilder()
            .withName("Arm Left")
            .withMotorControllerType(MotorControllerType.SPARK_MAX)
            .withID(7)
            .withIdleMode(IdleMode.kBrake)
            .withMotorType(MotorType.kBrushless)
            .withCurrentLimit(80)
            .withInverted(false)
            .withHomePosition(0.0)
            .withPositionConversion(1)
            .withVelocityConversion(1 / 60)
            .withKp(0.1)
            .build();

    public static final PositionSubsystemConstants kArmConstants =
        new PositionSubsystemConstantsBuilder()
            .withSuperstructureName("Arm")
            .withSubsystemName("Arm")
            .withLeaderConstants(kArmRight)
            .withFollowerConstants(kArmLeft)
            .withInitialState(ArmState.DOWN)
            .withSetpointTolerance(1)
            .withProfileConstraints(new TrapezoidProfile.Constraints(50, 100))
            .withManualControlMode(ManualControlMode.TRIGGERS)
            .withManualDeadband(.1)
            .withmanualMultiplier(1)
            .build();
  }
}
