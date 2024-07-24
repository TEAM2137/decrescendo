package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public final ShooterPivot pivot;
    public final ShooterFlywheel upperFlywheel, lowerFlywheel;

    public final InterpolatingDoubleTreeMap lookupTable = new InterpolatingDoubleTreeMap();

    public Shooter(ShooterPivotIO pivotIO, ShooterFlywheelIO upperFlywheelIO, ShooterFlywheelIO lowerFlywheelIO) {
        pivot = new ShooterPivot(pivotIO);
        upperFlywheel = new ShooterFlywheel(upperFlywheelIO);
        lowerFlywheel = new ShooterFlywheel(lowerFlywheelIO);

        // Add lookup table values here using:
        // lookupTable.put(distanceMeters, angleRads);
    }

    public Command aimAtDistance(double distanceMeters) {
        return pivot.setPositionTarget(Rotation2d.fromRadians(lookupTable.get(distanceMeters)))
            .andThen(upperFlywheel.setVelocity(300))
            .andThen(lowerFlywheel.setVelocity(300));
    }
}