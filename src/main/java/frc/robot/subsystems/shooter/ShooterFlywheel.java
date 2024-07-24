package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShooterFlywheel extends SubsystemBase {
    private final ShooterFlywheelIO io;
    private final ShooterFlywheelIOInputsAutoLogged inputs = new ShooterFlywheelIOInputsAutoLogged();

    private double targetVelocityRadPerSec;
    private boolean pidEnabled;
    private double setpointErrorThreshold = 3;

    public ShooterFlywheel(ShooterFlywheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }

    @AutoLogOutput
    public boolean atSetpoint() {
        return Math.abs(targetVelocityRadPerSec - inputs.velocityRadPerSec) < setpointErrorThreshold && pidEnabled;
    }

    public Trigger atSetpointTrigger() {
        return new Trigger(() -> atSetpoint());
    }

    public Command setVelocity(double velocityRPM) {
        return runOnce(() -> {
            targetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
            pidEnabled = true;
            io.setVelocityRPM(velocityRPM);
        });
    }

    public Command setVoltage(double volts) {
        return runOnce(() -> {
            pidEnabled = false;
            io.setVoltage(volts);
        });
    }
}
