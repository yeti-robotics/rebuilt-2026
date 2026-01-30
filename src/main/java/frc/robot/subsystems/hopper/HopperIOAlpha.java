package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class HopperIOAlpha implements HopperIO {
    public final TalonFX hopperRoller;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public HopperIOAlpha() {
        hopperRoller = new TalonFX(HopperConfigs.ROLLER_ID, Constants.rioBus);
        hopperRoller.getConfigurator().apply(HopperConfigs.TalonFXConfigs);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(hopperRoller);
        }
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.rollerSpeed = hopperRoller.getVelocity().getValueAsDouble();
    }

    @Override
    public void spinHopperRoller(double volts) {
        hopperRoller.setControl(new MotionMagicVelocityVoltage(volts));
    }

    @Override
    public void testSpinHopperRoller(double percent){
        hopperRoller.setControl(dutyCycleOut.withOutput(percent));
    }
}
