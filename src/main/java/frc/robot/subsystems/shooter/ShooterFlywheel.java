package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterFlywheel extends SubsystemBase {
  private final ShooterFlywheelIO io;
  private final ShooterFlywheelIOInputsAutoLogged inputs = new ShooterFlywheelIOInputsAutoLogged();

  private final String name;

  private double targetVelocityRadPerSec;
  private boolean pidEnabled;
  private double setpointErrorThreshold = 3;

  private final SysIdRoutine sysId;

  public ShooterFlywheel(ShooterFlywheelIO io, String name) {
    this.io = io;
    this.name = name;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("Shooter/" + name + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.recordOutput("Shooter/" + name + "/Voltage", inputs.appliedVolts);
    Logger.recordOutput("Shooter/" + name + "/VelocityRadPerSec", inputs.velocityRadPerSec);
    Logger.recordOutput("Shooter/" + name + "/Amps", inputs.currentAmps);
    Logger.recordOutput("Shooter/" + name + "/Setpoint", targetVelocityRadPerSec);
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  @AutoLogOutput
  public boolean atSetpoint() {
    return Math.abs(targetVelocityRadPerSec - inputs.velocityRadPerSec) < setpointErrorThreshold
        && pidEnabled;
  }

  public double getSetpoint() {
    return targetVelocityRadPerSec;
  }

  public Trigger atSetpointTrigger() {
    return new Trigger(() -> atSetpoint());
  }

  private void setVoltage(double volts) {
    io.setVoltage(volts);
    pidEnabled = false;
  }

  public Command runVelocity(double velocityRPM) {
    return runOnce(
        () -> {
          targetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
          pidEnabled = true;
          io.setVelocityRPM(velocityRPM);
        });
  }

  public Command runVoltage(double volts) {
    return runOnce(() -> setVoltage(volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
