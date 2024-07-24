package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivot extends SubsystemBase {
  private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();
  private final ShooterPivotIO io;

  public ShooterPivot(ShooterPivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command setVoltage(double volts) {
    return runOnce(() -> io.setVoltage(volts));
  }

  public Command setPositionTarget(Rotation2d target) {
    return runOnce(() -> io.setPositionTarget(target));
  }

  public Command setBrakeMode(boolean brake) {
    return runOnce(() -> io.setBrakeMode(brake));
  }
}
