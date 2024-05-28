package frc.lib.templates.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ClosedLoopTrapezoidProfileCommand extends TrapezoidProfileCommand {

  Supplier<Boolean> atSetpoint;

  public ClosedLoopTrapezoidProfileCommand(
      TrapezoidProfile profile,
      Consumer<State> output,
      Supplier<State> goal,
      Supplier<State> currentState,
      Supplier<Boolean> atSetpoint,
      Subsystem... requirements) {
    super(profile, output, goal, currentState, requirements);

    this.atSetpoint = atSetpoint;
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() && atSetpoint.get();
  }
}
