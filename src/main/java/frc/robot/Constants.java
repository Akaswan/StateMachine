package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.WristState;
import frc.robot.subsystems.manager.StatedSubsystem.CANSparkMaxConstants;
import frc.robot.subsystems.manager.StatedSubsystem.SubsystemConstants;
import frc.robot.subsystems.manager.StatedSubsystem.SubsystemType;

public class Constants {
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 0;
    }

    public static final class ArmConstants {

        public static final CANSparkMaxConstants kArmMasterConstants = new CANSparkMaxConstants();
        static {
            kArmMasterConstants.kID = 5;
            kArmMasterConstants.kIdleMode = IdleMode.kBrake;
            kArmMasterConstants.kMotorType = MotorType.kBrushless;
            kArmMasterConstants.kCurrentLimit = 80;
        }

        public static final CANSparkMaxConstants[] kArmSlaveConstants = new CANSparkMaxConstants[1];
        static {
            kArmSlaveConstants[0] = new CANSparkMaxConstants();
            kArmSlaveConstants[0].kID = 15;
            kArmSlaveConstants[0].kIdleMode = IdleMode.kBrake;
            kArmSlaveConstants[0].kMotorType = MotorType.kBrushless;
            kArmSlaveConstants[0].kCurrentLimit = 80;
        }

        public static final SubsystemConstants kArmConstants = new SubsystemConstants();
        static {
            kArmConstants.kName = "Arm";

            kArmConstants.kSubsystemType = SubsystemType.ARM;

            kArmConstants.kMasterConstants = kArmMasterConstants;
            kArmConstants.kSlaveConstants = kArmSlaveConstants;

            kArmConstants.kHomePosition = 0.0;
            kArmConstants.kRotationsPerUnitDistance = 360 / 100;

            kArmConstants.kKp = 0.2;
            kArmConstants.kKi = 0.0;
            kArmConstants.kKd = 0.0;
            kArmConstants.kSetpointTolerance = 0.1; 
            kArmConstants.kSmartMotionTolerance = 0.1;

            kArmConstants.kDefaultSlot = 0;

            kArmConstants.kMaxVelocity = 200;
            kArmConstants.kMaxAcceleration = 200;

            kArmConstants.kKs = 0.0;
            kArmConstants.kKg = 0.0;
            kArmConstants.kKv = 0.0;
            kArmConstants.kKa = 0.0;

            kArmConstants.kMaxPosition = 150.0;
            kArmConstants.kMinPosition = 0.0;

            kArmConstants.kManualAxis = XboxController.Axis.kRightY.value;
            kArmConstants.kManualMultiplier = 1;
            kArmConstants.kManualDeadZone = .1;

            kArmConstants.kInitialState = ArmState.HOME;
            kArmConstants.kManualState = ArmState.MANUAL;
            kArmConstants.kTransitionState = ArmState.TRANSITION;
            kArmConstants.kSetpointSwitchState = ArmState.SETPOINT_SWITCH;
        }
    }

    public static final class ElevatorConstants {

        public static final CANSparkMaxConstants kElevatorMasterConstants = new CANSparkMaxConstants();
        static {
            kElevatorMasterConstants.kID = 6;
            kElevatorMasterConstants.kIdleMode = IdleMode.kBrake;
            kElevatorMasterConstants.kMotorType = MotorType.kBrushless;
            kElevatorMasterConstants.kCurrentLimit = 80;
        }

        public static final CANSparkMaxConstants[] kElevatorSlaveConstants = new CANSparkMaxConstants[1];
        static {
            kElevatorSlaveConstants[0] = new CANSparkMaxConstants();
            kElevatorSlaveConstants[0].kID = 16;
            kElevatorSlaveConstants[0].kIdleMode = IdleMode.kBrake;
            kElevatorSlaveConstants[0].kMotorType = MotorType.kBrushless;
            kElevatorSlaveConstants[0].kCurrentLimit = 80;
        }

        public static final SubsystemConstants kElevatorConstants = new SubsystemConstants();
        static {
            kElevatorConstants.kName = "Elevator";

            kElevatorConstants.kSubsystemType = SubsystemType.ELEVATOR;

            kElevatorConstants.kMasterConstants = kElevatorMasterConstants;
            kElevatorConstants.kSlaveConstants = kElevatorSlaveConstants;

            kElevatorConstants.kHomePosition = 0.0;
            kElevatorConstants.kRotationsPerUnitDistance = 10;

            kElevatorConstants.kKp = 0.2;
            kElevatorConstants.kKi = 0.0;
            kElevatorConstants.kKd = 0.0;
            kElevatorConstants.kSetpointTolerance = 0.1; 
            kElevatorConstants.kSmartMotionTolerance = 0.1;

            kElevatorConstants.kDefaultSlot = 0;

            kElevatorConstants.kMaxVelocity = 1;
            kElevatorConstants.kMaxAcceleration = .5;

            kElevatorConstants.kKs = 0.0;
            kElevatorConstants.kKg = 0.0;
            kElevatorConstants.kKv = 0.0;
            kElevatorConstants.kKa = 0.0;

            kElevatorConstants.kMaxPosition = 3.0;
            kElevatorConstants.kMinPosition = 0.0;

            kElevatorConstants.kManualAxis = XboxController.Axis.kLeftY.value;
            kElevatorConstants.kManualMultiplier = .05;
            kElevatorConstants.kManualDeadZone = .1;

            kElevatorConstants.kInitialState = ElevatorState.HOME;
            kElevatorConstants.kManualState = ElevatorState.MANUAL;
            kElevatorConstants.kTransitionState = ElevatorState.TRANSITION;
            kElevatorConstants.kSetpointSwitchState = ElevatorState.SETPOINT_SWITCH;
        }
    }

    public static final class WristConstants {

        public static final CANSparkMaxConstants kWristMasterConstants = new CANSparkMaxConstants();
        static {
            kWristMasterConstants.kID = 7;
            kWristMasterConstants.kIdleMode = IdleMode.kBrake;
            kWristMasterConstants.kMotorType = MotorType.kBrushless;
            kWristMasterConstants.kCurrentLimit = 80;
        }

        public static final CANSparkMaxConstants[] kWristSlaveConstants = new CANSparkMaxConstants[0];

        public static final SubsystemConstants kWristConstants = new SubsystemConstants();
        static {
            kWristConstants.kName = "Wrist";

            kWristConstants.kSubsystemType = SubsystemType.WRIST;

            kWristConstants.kMasterConstants = kWristMasterConstants;
            kWristConstants.kSlaveConstants = kWristSlaveConstants;

            kWristConstants.kHomePosition = 0.0;
            kWristConstants.kRotationsPerUnitDistance = 360 / 100;

            kWristConstants.kKp = 0.2;
            kWristConstants.kKi = 0.0;
            kWristConstants.kKd = 0.0;
            kWristConstants.kSetpointTolerance = 0.1; 
            kWristConstants.kSmartMotionTolerance = 0.1;

            kWristConstants.kDefaultSlot = 0;

            kWristConstants.kMaxVelocity = 150;
            kWristConstants.kMaxAcceleration = 140;

            kWristConstants.kKs = 0.0;
            kWristConstants.kKg = 0.0;
            kWristConstants.kKv = 0.0;
            kWristConstants.kKa = 0.0;

            kWristConstants.kMaxPosition = 160;
            kWristConstants.kMinPosition = -160;

            kWristConstants.kManualAxis = XboxController.Axis.kRightX.value;
            kWristConstants.kManualMultiplier = 1;
            kWristConstants.kManualDeadZone = .1;

            kWristConstants.kInitialState = WristState.HOME;
            kWristConstants.kManualState = WristState.MANUAL;
            kWristConstants.kTransitionState = WristState.TRANSITION;
            kWristConstants.kSetpointSwitchState = WristState.SETPOINT_SWITCH;
        }
    }

}
