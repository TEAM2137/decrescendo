package frc.robot.subsystems.arm.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmRollers extends SubsystemBase {
  private final ArmRollersIOInputsAutoLogged inputs = new ArmRollersIOInputsAutoLogged();
  private final ArmRollersIO io;

  public ArmRollers(ArmRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm/Rollers", inputs);
  }

  public Command setVoltage(double volts) {
    return runOnce(() -> io.setVoltage(volts));
  }
}
