package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public final ShooterPivot pivot;
  public final ShooterFlywheel upperFlywheel, lowerFlywheel;

  public final InterpolatingDoubleTreeMap lookupTable = new InterpolatingDoubleTreeMap();

  public Shooter(
      ShooterPivotIO pivotIO,
      ShooterFlywheelIO upperFlywheelIO,
      ShooterFlywheelIO lowerFlywheelIO) {
    pivot = new ShooterPivot(pivotIO);
    upperFlywheel = new ShooterFlywheel(upperFlywheelIO, "UpperFlywheel");
    lowerFlywheel = new ShooterFlywheel(lowerFlywheelIO, "LowerFlywheel");

    // Add lookup table values here using:
    // lookupTable.put(distanceMeters, angleRads);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/Pivot/Position", pivot.getPosition().getDegrees());
  }

  public Command aimAtDistance(double distanceMeters) {
    return pivot
        .setPositionTarget(Rotation2d.fromRadians(lookupTable.get(distanceMeters)))
        .andThen(upperFlywheel.runVelocity(300))
        .andThen(lowerFlywheel.runVelocity(300));
  }

  public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return upperFlywheel
        .sysIdQuasistatic(direction)
        .alongWith(lowerFlywheel.sysIdQuasistatic(direction));
  }

  public Command flywheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return upperFlywheel.sysIdDynamic(direction).alongWith(lowerFlywheel.sysIdDynamic(direction));
  }
}
