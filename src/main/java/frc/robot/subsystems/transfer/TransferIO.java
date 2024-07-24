package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.AutoLog;

public interface TransferIO {
  @AutoLog
  public static class TransferIOInputs {
    public double appliedVolts = 0;
    public double velocityRotationsPerSec = 0;
    public double currentAmps = 0;
    public boolean noteSensor = false;
  }

  public default void updateInputs(TransferIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
