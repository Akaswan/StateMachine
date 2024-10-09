package frc.lib.utilities;

import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.utilities.DeviceConstants.MotorConstants;

public class MotorConstantsBuilder {
  private MotorConstants constants;

  public MotorConstantsBuilder() {
    constants = new MotorConstants();
  }

  public MotorConstantsBuilder withName(String name) {
    constants.kName = name;
    return this;
  }

  public MotorConstantsBuilder withID(int id) {
    constants.kID = id;
    return this;
  }

  public MotorConstantsBuilder withIsIdleBreak(boolean isIdleBreak) {
    constants.kIsIdleBreak = isIdleBreak;
    return this;
  }

  public MotorConstantsBuilder withMotorType(MotorType motorType) {
    constants.kMotorType = motorType;
    return this;
  }

  public MotorConstantsBuilder withCurrentLimit(int currentLimit) {
    constants.kCurrentLimit = currentLimit;
    return this;
  }

  /**
   *
   *
   * <h3>WithInverted</h3>
   *
   * <p>If the motor is a leader, this will determine if its inverted.
   *
   * <p>If the motor is a follower, this will determine if its running in the opposite direction
   * from the leader
   */
  public MotorConstantsBuilder withInverted(boolean inverted) {
    constants.kInverted = inverted;
    return this;
  }

  public MotorConstantsBuilder withHomePosition(double position) {
    constants.kHomePosition = position;
    return this;
  }

  public MotorConstantsBuilder withPositionConversion(double conversion) {
    constants.kPositionConversionFactor = conversion;
    return this;
  }

  public MotorConstantsBuilder withVelocityConversion(double conversion) {
    constants.kVelocityConversionFactor = conversion;
    return this;
  }

  public MotorConstantsBuilder withMaxAcceleration(double acceleration) {
    constants.kMaxAcceleration = acceleration;
    return this;
  }

  public MotorConstantsBuilder withMaxVelocity(double velocity) {
    constants.kMaxVelocity = velocity;
    return this;
  }

  public MotorConstantsBuilder withKp(double kp) {
    constants.kKp = kp;
    return this;
  }

  public MotorConstantsBuilder withKi(double ki) {
    constants.kKi = ki;
    return this;
  }

  public MotorConstantsBuilder withKd(double kd) {
    constants.kKd = kd;
    return this;
  }

  public MotorConstantsBuilder withKff(double kff) {
    constants.kKff = kff;
    return this;
  }

  public MotorConstantsBuilder withKs(double ks) {
    constants.kKs = ks;
    return this;
  }

  public MotorConstantsBuilder withKg(double kg) {
    constants.kKg = kg;
    return this;
  }

  public MotorConstantsBuilder withKv(double kv) {
    constants.kKv = kv;
    return this;
  }

  public MotorConstantsBuilder withKa(double ka) {
    constants.kKa = ka;
    return this;
  }

  public MotorConstants build() {
    return constants;
  }
}
