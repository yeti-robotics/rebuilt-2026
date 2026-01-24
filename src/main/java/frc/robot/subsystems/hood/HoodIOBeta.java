package frc.robot.subsystems.hood;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class HoodIOBeta implements HoodIO {
    private final TalonFX hoodMotor;
    private final CANcoder hoodCANcoder;

    public HoodIOBeta() {
        hoodMotor = new TalonFX(HoodConfigs.HOOD_MOTOR_ID, Constants.rioBus);
        hoodCANcoder = new CANcoder(HoodConfigs.HOOD_CANCODER_ID, Constants.rioBus);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(hoodMotor, hoodCANcoder);
        }
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodPosition = hoodMotor.getPosition().getValueAsDouble();
        inputs.hoodVelocity = hoodMotor.getVelocity().getValueAsDouble();
    }
}
