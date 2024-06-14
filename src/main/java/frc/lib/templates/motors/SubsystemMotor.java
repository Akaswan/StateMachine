package frc.lib.templates.motors;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.templates.subsystems.PositionSubsystem;
import frc.lib.templates.subsystems.PositionSubsystem.PositionSubsystemState;
import frc.lib.templates.subsystems.SubsystemConstants.MotorConstants;
import java.util.function.Supplier;

public interface SubsystemMotor {
  void configureMotor(MotorConstants constants, Supplier<Double> feedForward);

  void configureMotor(MotorConstants constants, Object followMotor);

  double getPostion();

  double getVelocity();

  void setVoltage(double voltage);

  void runToPosition(double position);

  void runToVelocity(double velocity);

  Object getMotor();

  Command runToPositionWithProfile(
      TrapezoidProfile profile,
      PositionSubsystemState transitionState,
      Supplier<TrapezoidProfile.State> profileStartSetpoint,
      PositionSubsystemState desiredState,
      Supplier<Boolean> isAtSetpoint,
      Runnable preCommand,
      BooleanConsumer postCommand,
      PositionSubsystem subsystem);
}
