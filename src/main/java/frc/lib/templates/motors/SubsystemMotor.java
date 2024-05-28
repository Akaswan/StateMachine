package frc.lib.templates.motors;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.templates.subsystems.PositionSubsystem;
import frc.lib.templates.subsystems.PositionSubsystem.SubsystemState;
import frc.lib.templates.subsystems.SubsystemConstants.MotorConstants;
import java.util.function.Supplier;

public interface SubsystemMotor {
  void configureMotor(
      MotorConstants constants,
      Supplier<SubsystemState> desiredState,
      Supplier<Double> feedForward);

  double getPostion();

  double getVelocity();

  void runToPosition();

  void runToVelocity();

  Command runToPositionWithProfile(
      TrapezoidProfile profile,
      SubsystemState transitionState,
      Supplier<TrapezoidProfile.State> profileStartSetpoint,
      SubsystemState desiredState,
      Supplier<Boolean> isAtSetpoint,
      Runnable preCommand,
      BooleanConsumer postCommand,
      PositionSubsystem subsystem);
}
