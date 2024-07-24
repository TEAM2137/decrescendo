package frc.robot.subsystems.transfer;

public class TransferIOSim implements TransferIO {
    private double appliedVolts = 0;
    @Override
    public void updateInputs(TransferIOInputs inputs) {
        inputs.appliedVolts = appliedVolts;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }

    
}
