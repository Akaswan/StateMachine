package frc.lib.utilities;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class DeviceConstants {
  public static class MotorConstants {
    public String kName = "ERROR_ASSIGN_NAME";
    public int kID = 0;
    public boolean kIsIdleBreak = true;
    public MotorType kMotorType = MotorType.kBrushless;
    public int kCurrentLimit = 0;
    public boolean kInverted = false;
    public double kHomePosition = 0.0;
    public double kPositionConversionFactor = 1;
    public double kVelocityConversionFactor = 1;
    public double kMaxAcceleration = 0;
    public double kMaxVelocity = 0;
    public double kKp = 0.0;
    public double kKi = 0.0;
    public double kKd = 0.0;

    public double kKff = 0.0; // Really only use this for velocity control (Velocity feed forward)

    // If you want to use custom feedforward
    public double kKs = 0.0;
    public double kKg = 0.0;
    public double kKv = 0.0;
    public double kKa = 0.0;
  }
}
