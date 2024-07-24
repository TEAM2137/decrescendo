package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.pivot.ArmPivot;
import frc.robot.subsystems.arm.pivot.ArmPivotIO;
import frc.robot.subsystems.arm.rollers.ArmRollers;
import frc.robot.subsystems.arm.rollers.ArmRollersIO;

public class AmpArm extends SubsystemBase {
  public final ArmPivot pivot;
  public final ArmRollers rollers;

  public AmpArm(ArmPivotIO pivotIO, ArmRollersIO rollersIO) {
    pivot = new ArmPivot(pivotIO);
    rollers = new ArmRollers(rollersIO);
  }
}
