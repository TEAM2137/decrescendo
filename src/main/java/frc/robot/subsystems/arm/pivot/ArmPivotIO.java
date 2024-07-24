package frc.robot.subsystems.arm.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmPivotIO {
    @AutoLog
    public class ArmPivotIOInputs {
        public Rotation2d absoluteEncoderPosition = new Rotation2d();
        public Rotation2d motorEncoderPosition = new Rotation2d();

        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    @AutoLog
    public class ArmWristPivotIOInputs {
        public Rotation2d absoluteEncoderPosition = new Rotation2d();
        public Rotation2d motorEncoderPosition = new Rotation2d();
        
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(ArmPivotIOInputs armInputs, ArmWristPivotIOInputs wristInputs) {}

    public default void setArmVoltage(double volts) {}

    public default void setWristVoltage(double volts) {}

    public default void setArmPositionTarget(Rotation2d target) {}

    public default void setWristPositionTarget(Rotation2d target) {}

    public default void setArmBrakeMode(boolean brake) {}

    public default void setWristBrakeMode(boolean brake) {}
}
