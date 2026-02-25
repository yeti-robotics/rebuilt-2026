package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class HopperIOBeta implements HopperIO {
    public final TalonFX hopperRoller;
    public final CANrange topCanRange;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public HopperIOBeta() {
        hopperRoller = new TalonFX(HopperConfigsBeta.BETA_ROLLER_ID, Constants.rioBus);
        topCanRange = new CANrange(HopperConfigsBeta.TOP_CANRANGE_ID, Constants.rioBus);
        hopperRoller.getConfigurator().apply(HopperConfigsBeta.TalonFXConfigs);
        topCanRange.getConfigurator().apply(HopperConfigsBeta.TOP_CANRANGE_CONFIGS);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(hopperRoller);
        }
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.rollerSpeed = hopperRoller.getVelocity().getValueAsDouble();
        inputs.isFull = topCanRange.getIsDetected().getValue();
    }

    @Override
    public void spinHopperRoller(double volts) {
        hopperRoller.setControl(new MotionMagicVelocityVoltage(volts));
    }

    @Override
    public void applyPower(double percent) {
        hopperRoller.setControl(dutyCycleOut.withOutput(percent));
    }
}
