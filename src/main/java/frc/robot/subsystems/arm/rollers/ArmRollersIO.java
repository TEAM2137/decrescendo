package frc.robot.subsystems.arm.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface ArmRollersIO {
    @AutoLog
    public class ArmRollersIOInputs {
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(ArmRollersIOInputs inputs) {}

    public default void setVoltage(double volts) {}
    
    public default void setVelocityRadPerSec(double velocityRadPerSec) {}

    public default void setVelocityRPM(double velocityRPM) {}
}
