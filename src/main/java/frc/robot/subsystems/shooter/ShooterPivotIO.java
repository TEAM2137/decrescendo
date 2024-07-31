package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {
  @AutoLog
  public class ShooterPivotIOInputs {
    public Rotation2d absoluteEncoderPosition = new Rotation2d();
    public Rotation2d motorEncoderPosition = new Rotation2d();
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(ShooterPivotIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPositionTarget(Rotation2d target) {}

  public default void setBrakeMode(boolean brake) {}

  public default void setPID(double kP, double kI, double kD) {}
}
