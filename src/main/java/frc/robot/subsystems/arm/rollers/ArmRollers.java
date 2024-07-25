package frc.robot.subsystems.arm.rollers;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ArmRollers extends SubsystemBase {
  private final ArmRollersIOInputsAutoLogged inputs = new ArmRollersIOInputsAutoLogged();
  private final ArmRollersIO io;

  private final SysIdRoutine sysId;

  public ArmRollers(ArmRollersIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/Rollers/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm/Rollers", inputs);
  }

  public Command setVoltage(double volts) {
    return runOnce(() -> io.setVoltage(volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
