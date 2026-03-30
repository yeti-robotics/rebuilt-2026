package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class IndexerIOAlpha implements IndexerIO {
    public final TalonFX indexerRoller;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public IndexerIOAlpha() {
        indexerRoller = new TalonFX(IndexerConfigsAlpha.ROLLER_ID, Constants.rioBus);
        indexerRoller.getConfigurator().apply(IndexerConfigsAlpha.TalonFXConfigs);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(indexerRoller);
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.rollerSpeed = indexerRoller.getVelocity().getValueAsDouble();
        inputs.motorTemp = indexerRoller.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void spinIndexerRoller(double volts) {
        indexerRoller.setControl(new MotionMagicVelocityVoltage(volts));
    }

    @Override
    public void applyPower(double percent) {
        indexerRoller.setControl(dutyCycleOut.withOutput(percent));
    }
}
