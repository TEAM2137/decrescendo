package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

public class ArmPivotIOSim implements ArmPivotIO {
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          50,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), 2),
          Units.inchesToMeters(10),
          Units.degreesToRadians(0),
          Units.degreesToRadians(125),
          true,
          0);

  private SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          50,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), 2),
          Units.inchesToMeters(10),
          Units.degreesToRadians(0),
          Units.degreesToRadians(180),
          false,
          0);

  private ArmFeedforward armFF = new ArmFeedforward(0.3, 0.15, 0);
  private ArmFeedforward wristFF = new ArmFeedforward(0.3, 0, 0);

  private PIDController armPid = new PIDController(5, 0, 0);
  private PIDController wristPid = new PIDController(5, 0, 0);

  private boolean armPidActive;
  private boolean wristPidActive;

  private double armAppliedVolts = 0;
  private double wristAppliedVolts = 0;
  private Rotation2d armTarget = new Rotation2d();
  private Rotation2d wristTarget = new Rotation2d();

  @Override
  public void updateInputs(ArmPivotIOInputs armInputs, ArmWristPivotIOInputs wristInputs) {
    if (armPidActive) {
      double pidEffort = armPid.calculate(armSim.getAngleRads(), armTarget.getRadians());
      double ffEffort = armFF.calculate(armSim.getAngleRads(), 0);

      armAppliedVolts = ffEffort + pidEffort;
      armSim.setInputVoltage(armAppliedVolts);
    }

    if (wristPidActive) {
      double pidEffort = wristPid.calculate(wristSim.getAngleRads(), wristTarget.getRadians());
      double ffEffort = wristFF.calculate(wristSim.getAngleRads(), 0);

      wristAppliedVolts = ffEffort + pidEffort;
      wristSim.setInputVoltage(wristAppliedVolts);
    }

    if (DriverStation.isDisabled()) {
      armAppliedVolts = 0;
      wristAppliedVolts = 0;
    }

    armSim.update(0.02);
    wristSim.update(0.02);

    armInputs.absoluteEncoderPosition = Rotation2d.fromRadians(armSim.getAngleRads());
    armInputs.motorEncoderPosition = Rotation2d.fromRadians(armSim.getAngleRads());
    armInputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
    armInputs.appliedVolts = armAppliedVolts;
    armInputs.currentAmps = armSim.getCurrentDrawAmps();

    wristInputs.absoluteEncoderPosition = Rotation2d.fromRadians(wristSim.getAngleRads());
    wristInputs.motorEncoderPosition = Rotation2d.fromRadians(wristSim.getAngleRads());
    wristInputs.velocityRadPerSec = wristSim.getVelocityRadPerSec();
    wristInputs.appliedVolts = wristAppliedVolts;
    wristInputs.currentAmps = wristSim.getCurrentDrawAmps();

    Logger.recordOutput("Arm/Pivot/Arm Angle", Units.radiansToDegrees(armSim.getAngleRads()));
    Logger.recordOutput("Arm/Pivot/Wrist Angle", Units.radiansToDegrees(wristSim.getAngleRads()));
  }

  @Override
  public void setArmPositionTarget(Rotation2d target) {
    this.armTarget = target;
    armPidActive = true;
  }

  @Override
  public void setWristPositionTarget(Rotation2d target) {
    this.wristTarget = target;
    wristPidActive = true;
  }

  @Override
  public void setArmVoltage(double volts) {
    armSim.setInputVoltage(volts);
    armAppliedVolts = volts;
    armPidActive = false;
  }

  @Override
  public void setWristVoltage(double volts) {
    wristSim.setInputVoltage(volts);
    wristAppliedVolts = volts;
    wristPidActive = false;
  }
}
