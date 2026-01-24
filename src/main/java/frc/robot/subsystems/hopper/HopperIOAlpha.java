package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class HopperIOAlpha implements HopperIO {
    private final TalonFX hopperMotor;

    public HopperIOAlpha() {
        hopperMotor = new TalonFX(HopperConfigs.HOPPER_MOTOR_ID, Constants.rioBus);
        hopperMotor.getConfigurator().apply(HopperConfigs.TalonFXConfigs);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(hopperMotor);
        }
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.rollerSpeed = hopperMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void spinHopperRoller(double volts) {
        hopperMotor.setControl(new MotionMagicVelocityVoltage(volts));
    }
}
