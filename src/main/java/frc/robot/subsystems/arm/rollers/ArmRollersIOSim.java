package frc.robot.subsystems.arm.rollers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.littletonrobotics.junction.Logger;

public class ArmRollersIOSim implements ArmRollersIO {
  private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getKrakenX60(1), 2, 0.004);

  private PIDController pid = new PIDController(2.5, 0, 0);
  private boolean pidActive;

  private double appliedVolts = 0;
  private double targetVelocityRadPerSec;

  @Override
  public void updateInputs(ArmRollersIOInputs inputs) {
    if (pidActive) {
      appliedVolts =
          pid.calculate(flywheelSim.getAngularVelocityRadPerSec(), targetVelocityRadPerSec);
      flywheelSim.setInputVoltage(appliedVolts);
    }

    if (DriverStation.isDisabled()) {
      appliedVolts = 0;
    }

    flywheelSim.update(0.02);

    inputs.velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = flywheelSim.getCurrentDrawAmps();

    Logger.recordOutput("Arm/Rollers/Position", 0);
    Logger.recordOutput("Arm/Rollers/VelocityRadPerSec", flywheelSim.getAngularVelocityRadPerSec());
  }

  @Override
  public void setVoltage(double volts) {
    flywheelSim.setInputVoltage(volts);
    appliedVolts = volts;
    pidActive = false;
  }

  @Override
  public void setVelocityRadPerSec(double velocityRadPerSec) {
    targetVelocityRadPerSec = velocityRadPerSec;
    pidActive = true;
  }

  @Override
  public void setVelocityRPM(double velocityRPM) {
    setVelocityRadPerSec(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
  }
}
