package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterPivotIOSim implements ShooterPivotIO {
  private SingleJointedArmSim pivot =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          50,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), 2),
          Units.inchesToMeters(10),
          0,
          Units.degreesToRadians(45),
          true,
          0);

  private ArmFeedforward ff = new ArmFeedforward(0.3, 1, 0);
  private PIDController pid = new PIDController(3, 0, 0);
  private boolean pidEnabled;

  private double appliedVolts = 0;
  private Rotation2d target = new Rotation2d();

  public ShooterPivotIOSim() {}

  @Override
  public void updateInputs(ShooterPivotIOInputs inputs) {
    if (pidEnabled) {
      double pidEffort = pid.calculate(pivot.getAngleRads(), target.getRadians());
      double ffEffort = ff.calculate(pivot.getAngleRads(), 0);

      appliedVolts = ffEffort + pidEffort;
      pivot.setInputVoltage(appliedVolts);
    }

    pivot.update(0.02);

    inputs.absoluteEncoderPosition = Rotation2d.fromRadians(pivot.getAngleRads());
    inputs.motorEncoderPosition = Rotation2d.fromRadians(pivot.getAngleRads());
    inputs.velocityRadPerSec = pivot.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = pivot.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    pivot.setInputVoltage(volts);
    appliedVolts = volts;
    pidEnabled = false;
  }

  @Override
  public void setPositionTarget(Rotation2d target) {
    this.target = target;
    pidEnabled = true;
  }
}
