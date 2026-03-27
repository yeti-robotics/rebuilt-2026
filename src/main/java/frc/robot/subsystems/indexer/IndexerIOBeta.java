package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class IndexerIOBeta implements IndexerIO {
    public final TalonFX indexerRoller;
    public final CANrange topCanRange;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public IndexerIOBeta() {
        indexerRoller = new TalonFX(IndexerConfigsBeta.BETA_ROLLER_ID, Constants.rioBus);
        topCanRange = new CANrange(IndexerConfigsBeta.TOP_CANRANGE_ID, Constants.rioBus);
        indexerRoller.getConfigurator().apply(IndexerConfigsBeta.TalonFXConfigs);
        topCanRange.getConfigurator().apply(IndexerConfigsBeta.TOP_CANRANGE_CONFIGS);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(indexerRoller);
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.rollerSpeed = indexerRoller.getVelocity().getValueAsDouble();
        inputs.isFull = topCanRange.getIsDetected().getValue();
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
