package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmPivot extends SubsystemBase {
  private final ArmPivotIOInputsAutoLogged armInputs = new ArmPivotIOInputsAutoLogged();
  private final ArmWristPivotIOInputsAutoLogged wristInputs = new ArmWristPivotIOInputsAutoLogged();
  private final ArmPivotIO io;

  public ArmPivot(ArmPivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(armInputs, wristInputs);
    Logger.processInputs("Arm/Arm Pivot", armInputs);
    Logger.processInputs("Arm/Wrist Pivot", wristInputs);
  }

  public Command setArmVoltage(double volts) {
    return runOnce(() -> io.setArmVoltage(volts));
  }

  public Command setWristVoltage(double volts) {
    return runOnce(() -> io.setWristVoltage(volts));
  }

  public Command setArmPositionTarget(Rotation2d target) {
    return runOnce(() -> io.setArmPositionTarget(target));
  }

  public Command setWristPositionTarget(Rotation2d target) {
    return runOnce(() -> io.setWristPositionTarget(target));
  }

  public Command setArmBrakeMode(boolean brake) {
    return runOnce(() -> io.setArmBrakeMode(brake));
  }

  public Command setWristBrakeMode(boolean brake) {
    return runOnce(() -> io.setWristBrakeMode(brake));
  }
}
