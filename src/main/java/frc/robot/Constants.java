package frc.robot;

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
            kArmSlaveConstants[0].kID = 15;
            kArmSlaveConstants[0].kIdleMode = IdleMode.kBrake;
            kArmSlaveConstants[0].kMotorType = MotorType.kBrushless;
            kArmSlaveConstants[0].kCurrentLimit = 80;
        }

        public static final ServoMotorSubsystemConstants kArmConstants = new ServoMotorSubsystemConstants()
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

            kArmConstants.kMaxVelocity = 75;
            kArmConstants.kMaxAcceleration = 60;

            kArmConstants.kKs = 0.0;
            kArmConstants.kKg = 0.0;
            kArmConstants.kKv = 0.0;
            kArmConstants.kKa = 0.0;

            kArmConstants.kMaxPosition = 500.0;
            kArmConstants.kMinPosition = 0.0;

            kArmConstants.kinitialState = null;
        }
    }
}
