package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim implements ClimberIO {
  ElevatorSim sim;
  private double appliedVolts = 0;
  private double targetPosition = 0;

  private boolean isPID = false;

  private ElevatorFeedforward feedforward;
  private PIDController pidController;


  // -7 weight is broken :[
  public ClimberIOSim() {
    feedforward = new ElevatorFeedforward(0, 0, 0);
    pidController = new PIDController(0, 0, 0);
    sim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            50,
            -7,
            Units.inchesToMeters(0.75),
            0,
            Units.feetToMeters(3),
            true,
            0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (isPID) {
      double ff = feedforward.calculate(0);
      double pid = pidController.calculate(sim.getPositionMeters(), targetPosition);

      appliedVolts = ff + pid;
    }

    if (DriverStation.isDisabled()) {
      appliedVolts = 0;
    }

    sim.setInput(appliedVolts);
    sim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.positionMeters = sim.getPositionMeters();
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }
}
