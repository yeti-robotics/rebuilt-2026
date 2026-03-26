package frc.robot.subsystems.powerdist;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerDistributorPDP implements PowerDistributorIO {
    private final PowerDistribution pdp;

    public PowerDistributorPDP() {
        pdp = new PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);
    }

    @Override
    public void updateInputs(PowerDistributorIOInputs inputs) {
        inputs.totalCurrent = pdp.getTotalCurrent();
        inputs.totalVoltage = pdp.getVoltage();
        inputs.channelCurrents = pdp.getAllCurrents();
        inputs.temperature = pdp.getTemperature();
    }
}
