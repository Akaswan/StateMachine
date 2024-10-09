// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SubsystemStateManager;
import frc.robot.subsystems.SubsystemStateManager.RobotState;
import frc.robot.subsystems.amper.AmperElevator;
import frc.robot.subsystems.amper.AmperWheels;
import frc.robot.subsystems.climber.ClimberWinch;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.intake.TransferWheels;
import frc.robot.subsystems.launcher.LauncherFlywheels;
import frc.robot.subsystems.launcher.LauncherPivot;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // CONTROLLERS \\
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // SHUFFLEBOARD TABS \\
  public static ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");

  // SUBSYSTEMS \\
  private final IntakeWrist m_intakeWrist = new IntakeWrist();
  private final AmperElevator m_amperElevator = new AmperElevator();
  private final TransferWheels m_transferWheels = new TransferWheels();
  private final ClimberWinch m_climberWinch = new ClimberWinch();
  private final LauncherPivot m_launcherPivot = new LauncherPivot();
  private final AmperWheels m_amperWheels = new AmperWheels();
  private final IntakeWheels m_intakeWheels = new IntakeWheels();
  private final LauncherFlywheels m_launcherFlywheels = new LauncherFlywheels();

  // SENDABLE CHOOSER \\
  public static LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    // NAMED COMMANDS FOR AUTO \\

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

    autoChooser.addDefaultOption("None", new InstantCommand());

    m_mainTab
        .add(autoChooser.getSendableChooser())
        .withPosition(0, 0)
        .withSize(2, 1)
        .withWidget(BuiltInWidgets.kComboBoxChooser);

    // CONFIGURE DEFAULT COMMANDS \\

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_driverController
        .a()
        .onTrue(SubsystemStateManager.intake(m_intakeWrist, m_intakeWheels, m_transferWheels));
    m_driverController
        .b()
        .onTrue(SubsystemStateManager.stow(m_intakeWrist, m_intakeWheels, m_transferWheels));

    new Trigger(() -> SubsystemStateManager.getCurrentState() == RobotState.INTAKING)
        .and(m_transferWheels::getIsNoteInTransfer)
        .onTrue(SubsystemStateManager.stow(m_intakeWrist, m_intakeWheels, m_transferWheels));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
