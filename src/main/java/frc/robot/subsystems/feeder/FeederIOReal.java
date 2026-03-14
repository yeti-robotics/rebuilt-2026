package frc.robot.subsystems.feeder;

import static frc.robot.constants.Constants.currentMode;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class FeederIOReal implements FeederIO {
    public final TalonFX feederMotor;
    public final CANrange feederSensor;

    public final MotionMagicVelocityVoltage velocityVoltageRequest = new MotionMagicVelocityVoltage(0);
    public final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    public FeederIOReal() {
        if (currentMode == Constants.Mode.ALPHA) {
            feederMotor = new TalonFX(FeederConfigsAlpha.FEEDER_MOTOR_ID, Constants.rioBus);
            feederMotor.getConfigurator().apply(FeederConfigsAlpha.FEEDER_MOTOR_CONFIGS);

            feederSensor = new CANrange(FeederConfigsAlpha.FEEDER_CANRANGE_ID, Constants.rioBus);
            feederSensor.getConfigurator().apply(FeederConfigsAlpha.CANRANGE_CONFIGS);

        } else {
            feederMotor = new TalonFX(FeederConfigsBeta.FEEDER_MOTOR_ID, Constants.rioBus);
            feederMotor.getConfigurator().apply(FeederConfigsBeta.FEEDER_MOTOR_CONFIGS);

            feederSensor = new CANrange(FeederConfigsBeta.FEEDER_CANRANGE_ID, Constants.rioBus);
            feederSensor.getConfigurator().apply(FeederConfigsBeta.CANRANGE_CONFIGS);
        }
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(feederMotor);
        }
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.feederVoltage = feederMotor.getMotorVoltage().getValueAsDouble();
        inputs.isDetected = feederSensor.getIsDetected().getValue();
    }

    @Override
    public void spinFeeder(double volts) {
        feederMotor.setControl(velocityVoltageRequest.withVelocity(volts));
    }

    @Override
    public void applyPower(double percent) {
        feederMotor.setControl(dutyRequest.withOutput(percent));
    }
}
