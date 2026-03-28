package frc.robot.subsystems.powerdist;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerDistributorReal implements PowerDistributorIO {
    private final PowerDistribution pdh;

    public PowerDistributorReal(int module, PowerDistribution.ModuleType type) {
        pdh = new PowerDistribution(module, type);
    }

    @Override
    public void updateInputs(PowerDistributorIOInputs inputs) {
        inputs.totalCurrent = pdh.getTotalCurrent();
        inputs.totalVoltage = pdh.getVoltage();
        inputs.channelCurrents = pdh.getAllCurrents();
        inputs.temperature = pdh.getTemperature();
    }
}
