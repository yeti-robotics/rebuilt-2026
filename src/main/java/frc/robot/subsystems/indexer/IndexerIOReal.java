package frc.robot.subsystems.indexer;

import static frc.robot.constants.Constants.currentMode;
import static frc.robot.subsystems.indexer.IndexerConfigsAlpha.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class IndexerIOReal implements IndexerIO {
    public final TalonFX indexerMotor;
    public final CANrange indexerSensor;

    public final MotionMagicVelocityVoltage velocityVoltageRequest = new MotionMagicVelocityVoltage(0);
    public final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    public IndexerIOReal() {
        if(currentMode == Constants.Mode.ALPHA) {
            indexerMotor = new TalonFX(IndexerConfigsAlpha.INDEXER_MOTOR_ID, Constants.rioBus);
            indexerMotor.getConfigurator().apply(IndexerConfigsAlpha.INDEXER_MOTOR_CONFIGS);

            indexerSensor = new CANrange(IndexerConfigsAlpha.INDEXER_CANRANGE_ID, Constants.rioBus);
            indexerSensor.getConfigurator().apply(IndexerConfigsAlpha.CANRANGE_CONFIGS);

            if (Robot.isSimulation()) {
                PhysicsSim.getInstance().addTalonFX(indexerMotor);
            }
        }
        else{
            indexerMotor = new TalonFX(IndexerConfigsBeta.INDEXER_MOTOR_ID, Constants.rioBus);
            indexerMotor.getConfigurator().apply(IndexerConfigsBeta.INDEXER_MOTOR_CONFIGS);

            indexerSensor = new CANrange(IndexerConfigsBeta.INDEXER_CANRANGE_ID, Constants.rioBus);
            indexerSensor.getConfigurator().apply(IndexerConfigsBeta.CANRANGE_CONFIGS);

            if (Robot.isSimulation()) {
                PhysicsSim.getInstance().addTalonFX(indexerMotor);
            }
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVoltage = indexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.isDetected = indexerSensor.getIsDetected().getValue();
    }

    @Override
    public void spinIndexer(double volts) {
        indexerMotor.setControl(velocityVoltageRequest.withVelocity(volts));
    }

    @Override
    public void applyPower(double percent) {
        indexerMotor.setControl(dutyRequest.withOutput(percent));
    }
}
