package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterFlywheelIOSim implements ShooterFlywheelIO {
  private FlywheelSim flywheel = new FlywheelSim(DCMotor.getKrakenX60(1), 1.5, 0.004);

  private double appliedVolts;
  private double targetVelocityRadPerSec;

  private boolean pidEnabled;
  private PIDController pid = new PIDController(0.037146, 0, 0);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(-0.009476, .028492, .0045156);

  public ShooterFlywheelIOSim() {}

  @Override
  public void updateInputs(ShooterFlywheelIOInputs inputs) {
    if (pidEnabled) {
      double pidEffort =
          pid.calculate(flywheel.getAngularVelocityRadPerSec(), targetVelocityRadPerSec);
      double ffEffort = ff.calculate(targetVelocityRadPerSec);
      
      appliedVolts = pidEffort + ffEffort;
      flywheel.setInput(pidEffort);
    }

    flywheel.update(0.02);

    inputs.velocityRadPerSec = flywheel.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = flywheel.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    flywheel.setInput(volts);
    appliedVolts = volts;
    pidEnabled = false;
  }

  @Override
  public void setVelocityRadPerSec(double velocityRadPerSec) {
    targetVelocityRadPerSec = velocityRadPerSec;
    pidEnabled = true;
  }

  @Override
  public void setVelocityRPM(double velocityRPM) {
    setVelocityRadPerSec(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
  }
}
