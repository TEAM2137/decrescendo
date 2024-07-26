package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelIO {
  @AutoLog
  public class ShooterFlywheelIOInputs {
    public double velocityRadPerSec;
    public double appliedVolts;
    public double currentAmps;
  }

  public default void updateInputs(ShooterFlywheelIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocityRadPerSec(double velocityRadPerSec) {}

  public default void setVelocityRPM(double velocityRPM) {}
}