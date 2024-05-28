// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  boolean autonInitCommandRun = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  @SuppressWarnings("resource")
  public void robotInit() {

    Logger.recordMetadata("ProjectName", "StateMachine"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }

    Logger.addDataReceiver(new NT4Publisher());

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the
    // "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    m_robotContainer = new RobotContainer();

    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    SignalLogger.enableAutoLogging(false);

    // Command disabledCommand = new PathPlannerAuto("Dummy-Auto-Fix-Auto").ignoringDisable(true);
    // disabledCommand.schedule();
    // Timer.delay(1);
    // disabledCommand.cancel();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = new WaitCommand(0.01).andThen(m_robotContainer.getAutonomousCommand());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {

      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    if (autonInitCommandRun == false) {
      Command autonInitCommand = FollowPathCommand.warmupCommand();
      // new PathPlannerAuto("1 Meter Auto").ignoringDisable(true);
      autonInitCommand.schedule();
      autonInitCommandRun = true;
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
