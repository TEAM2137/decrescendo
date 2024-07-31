package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterPivot extends SubsystemBase {
  private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();
  private final ShooterPivotIO io;

  private Rotation2d targetPosition = Rotation2d.fromDegrees(0);

  public ShooterPivot(ShooterPivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.recordOutput("Shooter/Pivot/Setpoint", targetPosition.getDegrees());
    Logger.recordOutput("Shooter/Pivot/Position", getPosition().getDegrees());
    Logger.recordOutput("Shooter/Pivot/Voltage", inputs.appliedVolts);
  }

  public Rotation2d getPosition() {
    return inputs.absoluteEncoderPosition;
  }

  public Command setVoltage(double volts) {
    return runOnce(() -> io.setVoltage(volts));
  }

  public Command setPositionTarget(Rotation2d target) {
    return runOnce(
        () -> {
          io.setPositionTarget(target);
          targetPosition = target;
        });
  }

  public Command setBrakeMode(boolean brake) {
    return runOnce(() -> io.setBrakeMode(brake));
  }
}
