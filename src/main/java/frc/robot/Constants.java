package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.manager.ServoMotorSubsystem.CANSparkMaxConstants;
import frc.robot.subsystems.manager.ServoMotorSubsystem.ServoMotorSubsystemConstants;

public class Constants {
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
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

        public static final ServoMotorSubsystemConstants kArmConstants = new ServoMotorSubsystemConstants();
        static {
            kArmConstants.kName = "Arm";

            kArmConstants.kMasterConstants = kArmMasterConstants;
            kArmConstants.kSlaveConstants = kArmSlaveConstants;

            kArmConstants.kHomePosition = 0.0;
            kArmConstants.kRotationsPerUnitDistance = 360 / 100;

            kArmConstants.kKp = 0.2;
            kArmConstants.kKi = 0.0;
            kArmConstants.kKd = 0.0;
            kArmConstants.kSetpointTolerance = 0.25; 
            kArmConstants.kSmartMotionTolerance = 0.25;

            kArmConstants.kDefaultSlot = 0;

            kArmConstants.kMaxVelocity = .01;
            kArmConstants.kMaxAcceleration = .005;

            kArmConstants.kKs = 0.0;
            kArmConstants.kKg = 0.0;
            kArmConstants.kKv = 0.0;
            kArmConstants.kKa = 0.0;

            kArmConstants.kMaxPosition = 500.0;
            kArmConstants.kMinPosition = 0.0;

            kArmConstants.kinitialState = ArmState.HOME;
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

        public static final ServoMotorSubsystemConstants kElevatorConstants = new ServoMotorSubsystemConstants();
        static {
            kElevatorConstants.kName = "Elevator";

            kElevatorConstants.kMasterConstants = kArmMasterConstants;
            kElevatorConstants.kSlaveConstants = kArmSlaveConstants;

            kElevatorConstants.kHomePosition = 0.0;
            kElevatorConstants.kRotationsPerUnitDistance = 360 / 100;

            kElevatorConstants.kKp = 0.2;
            kElevatorConstants.kKi = 0.0;
            kElevatorConstants.kKd = 0.0;
            kElevatorConstants.kSetpointTolerance = 0.25; 
            kElevatorConstants.kSmartMotionTolerance = 0.25;

            kElevatorConstants.kDefaultSlot = 0;

            kElevatorConstants.kMaxVelocity = .01;
            kElevatorConstants.kMaxAcceleration = .005;

            kElevatorConstants.kKs = 0.0;
            kElevatorConstants.kKg = 0.0;
            kElevatorConstants.kKv = 0.0;
            kElevatorConstants.kKa = 0.0;

            kElevatorConstants.kMaxPosition = 500.0;
            kElevatorConstants.kMinPosition = 0.0;

            kElevatorConstants.kinitialState = ArmState.HOME;
        }
    }

}
