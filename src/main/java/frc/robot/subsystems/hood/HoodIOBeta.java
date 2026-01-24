package frc.robot.subsystems.hood;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class HoodIOBeta implements HoodIO {
    private final TalonFX hoodMotor;
    private final CANcoder hoodCANcoder;
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0.0);

    public HoodIOBeta() {
        hoodMotor = new TalonFX(HoodConfigs.HOOD_MOTOR_ID, Constants.rioBus);
        hoodCANcoder = new CANcoder(HoodConfigs.HOOD_CANCODER_ID, Constants.rioBus);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(hoodMotor, hoodCANcoder);
        }

        hoodMotor.getConfigurator().apply(HoodConfigs.HOOD_MOTOR_CONFIGS);
        hoodCANcoder.getConfigurator().apply(HoodConfigs.HOOD_CANCODER_CONFIGS);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodPosition = hoodMotor.getPosition().getValueAsDouble();
        inputs.hoodVelocity = hoodMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void moveToPosition(double position) {
        hoodMotor.setControl(positionRequest.withPosition(position));
    }
}
