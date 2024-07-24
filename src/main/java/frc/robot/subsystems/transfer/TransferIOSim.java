package frc.robot.subsystems.transfer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TransferIOSim implements TransferIO {
  DCMotorSim sim;

  private double appliedVolts = 0;
  private BooleanEntry noteSensorEntry;

  public TransferIOSim() {
    sim = new DCMotorSim(DCMotor.getNEO(1), 25, 0.02);

    noteSensorEntry =
        NetworkTableInstance.getDefault()
            .getTable("SimInputs")
            .getBooleanTopic("NoteSensor")
            .getEntry(false);
    noteSensorEntry.setDefault(false);
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      appliedVolts = 0;
    }

    sim.setInput(appliedVolts);
    sim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.velocityRotationsPerSec = sim.getAngularVelocityRPM() / 60;
    inputs.noteSensor = noteSensorEntry.get();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
