package frc.robot.subsystems.powerdist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributor extends SubsystemBase {
    private final PowerDistributorIO io;
    private final PowerDistributorIOInputsAutoLogged inputs = new PowerDistributorIOInputsAutoLogged();

    public PowerDistributor(PowerDistributorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getTemperature() {
        return inputs.temperature;
    }

    public double getTotalCurrent() {
        return inputs.totalCurrent;
    }

    public double getTotalVoltage() {
        return inputs.totalVoltage;
    }

    public double[] getChannelCurrents() {
        return inputs.channelCurrents;
    }

    public double getCurrent(int channel) {
        return inputs.channelCurrents[channel];
    }
}
