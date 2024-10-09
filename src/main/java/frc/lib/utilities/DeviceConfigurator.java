package frc.lib.utilities;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.lib.utilities.DeviceConstants.MotorConstants;

public class DeviceConfigurator {
  //   public static void configureSparkMaxSteerMotor(CANSparkMax motor) {
  //     RelativeEncoder encoder = motor.getEncoder();
  //     SparkPIDController controller = motor.getPIDController();

  //     motor.setInverted(true);
  //     motor.setSmartCurrentLimit(40);
  //     motor.setIdleMode(IdleMode.kBrake);

  //     encoder.setPositionConversionFactor(DriveConstants.kTurnRotationsToDegrees);

  //     controller.setP(DriveConstants.turnkp);
  //     controller.setI(DriveConstants.turnki);
  //     controller.setD(DriveConstants.turnkd);
  //     controller.setFF(DriveConstants.turnkff);
  //   }

  //   public static void configureSparkFlexDriveMotor(CANSparkFlex motor) {
  //     RelativeEncoder encoder = motor.getEncoder();
  //     SparkPIDController controller = motor.getPIDController();

  //     motor.setInverted(true);
  //     motor.setSmartCurrentLimit(80);
  //     motor.setIdleMode(IdleMode.kBrake);
  //     motor.setOpenLoopRampRate(DriveConstants.driverampRate);

  //     encoder.setPositionConversionFactor(DriveConstants.kDriveRevToMeters);
  //     encoder.setVelocityConversionFactor(DriveConstants.kDriveRpmToMetersPerSecond);

  //     controller.setP(DriveConstants.drivekp);
  //     controller.setI(DriveConstants.driveki);
  //     controller.setD(DriveConstants.drivekd);
  //     controller.setFF(DriveConstants.drivekff);
  //   }

  //   public static void configureAngleEncoder(CANcoder encoder) {
  //     CANcoderConfiguration configuration = new CANcoderConfiguration();

  //     encoder.getConfigurator().apply(configuration);

  //     configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
  //     configuration.MagnetSensor.MagnetOffset = 0;
  //     configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

  //     encoder.getConfigurator().apply(configuration);
  //   }

  public static void configureTalonFX(TalonFX motor, MotorConstants constants) {
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = constants.kCurrentLimit;
    configuration.CurrentLimits.SupplyCurrentLimit = constants.kCurrentLimit;

    configuration.Feedback.SensorToMechanismRatio = constants.kPositionConversionFactor;

    configuration.MotionMagic.MotionMagicAcceleration = constants.kMaxAcceleration;
    configuration.MotionMagic.MotionMagicCruiseVelocity = constants.kMaxVelocity;

    configuration.MotorOutput.Inverted =
        constants.kInverted == true
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    configuration.MotorOutput.NeutralMode =
        constants.kIsIdleBreak == true ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    configuration.Slot0.kA = constants.kKa;
    configuration.Slot0.kS = constants.kKs;
    configuration.Slot0.kV = constants.kKv;
    configuration.Slot0.kG = constants.kKg;
    configuration.Slot0.kP = constants.kKp;
    configuration.Slot0.kI = constants.kKi;
    configuration.Slot0.kD = constants.kKd;

    motor.getConfigurator().apply(configuration);
    motor.setPosition(constants.kHomePosition);
  }

  public static void configureSpark(
      CANSparkBase motor,
      RelativeEncoder encoder,
      SparkPIDController controller,
      MotorConstants constants) {

    IdleMode idleMode = constants.kIsIdleBreak == true ? IdleMode.kBrake : IdleMode.kCoast;

    motor.setIdleMode(idleMode);
    motor.setSmartCurrentLimit(constants.kCurrentLimit);
    motor.setInverted(constants.kInverted);

    controller.setP(constants.kKp);
    controller.setI(constants.kKi);
    controller.setD(constants.kKd);

    encoder.setPosition(constants.kHomePosition);
    encoder.setPositionConversionFactor(constants.kPositionConversionFactor);
    encoder.setVelocityConversionFactor(constants.kPositionConversionFactor / 60);

    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65534);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65534);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65534);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65534);

    motor.burnFlash();
  }
}
