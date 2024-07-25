package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionMeters = 0;
    public double appliedVolts = 0;
    public double velocityMetersPerSec = 0;
    public double currentAmps = 0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(double meters) {}
}
