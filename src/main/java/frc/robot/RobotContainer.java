// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivebase.TeleopSwerve;
import frc.robot.commands.superstructure.SetLEDEffect;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED.RGB;
import frc.robot.subsystems.LED.effects.timedeffects.ListEffect;
import frc.robot.subsystems.LED.effects.timedeffects.RainbowEffect;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDrive.DriveMode;

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
  public static final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // SHUFFLEBOARD TABS \\
  public static ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");

  public static final LED m_LED = LED.getInstance();

  public static SwerveDrive m_drivebase = SwerveDrive.getInstance();

  public static Arm m_arm = Arm.getInstance();

  // SENDABLE CHOOSER \\
  public static SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // NAMED COMMANDS FOR AUTO \\

    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("None", new InstantCommand());

    m_mainTab
        .add(autoChooser)
        .withPosition(0, 0)
        .withSize(2, 1)
        .withWidget(BuiltInWidgets.kComboBoxChooser);

    // CONFIGURE DEFAULT COMMANDS \\
    m_drivebase.setDefaultCommand(
        new TeleopSwerve(
            m_driverController,
            OperatorConstants.kThrottleAxis,
            OperatorConstants.kStrafeAxis,
            OperatorConstants.kSteerAxis,
            OperatorConstants.kPercentModifier,
            false,
            true));
    m_arm.setDefaultCommand(Commands.run(m_arm::manualControl, m_arm));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_driverController.start().onTrue(new InstantCommand(() -> m_drivebase.setGyro(0)));
    m_driverController
        .x()
        .onTrue(new InstantCommand(() -> m_drivebase.setDriveMode(DriveMode.XWHEELS)));

    // m_operatorController.a().onTrue(m_arm.moveWithProfile(ArmState.UP, m_arm));
    // m_operatorController.b().onTrue(m_arm.moveWithProfile(ArmState.DOWN, m_arm));

    m_operatorController
        .a()
        .onTrue(
            new SetLEDEffect(
                new ListEffect(
                    .005,
                    new RGB(3, 252, 57),
                    LEDConstants.kArmLeftSegment,
                    LEDConstants.kSnakeEffectBlueprint)));
    m_operatorController
        .b()
        .onTrue(new SetLEDEffect(new RainbowEffect(.02, LEDConstants.kArmRightSegment)));

    m_operatorController
        .x()
        .onTrue(
            new SetLEDEffect(
                new ListEffect(
                    .01,
                    new RGB(3, 252, 57),
                    LEDConstants.kArmRightSegment,
                    LEDConstants.kSnakeEffectBlueprint,
                    true,
                    .25,
                    1)));
    m_operatorController
        .y()
        .onTrue(new SetLEDEffect(new RainbowEffect(.01, LEDConstants.kArmLeftSegment)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
