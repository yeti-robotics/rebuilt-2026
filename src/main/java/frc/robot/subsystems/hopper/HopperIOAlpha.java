package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class HopperIOAlpha implements HopperIO {
    public final TalonFX hopperRoller;

    public HopperIOAlpha() {
        hopperRoller = new TalonFX(HopperConfigs.ROLLER_ID, Constants.rioBus);
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
        hopperRoller.setControl(new VoltageOut(volts));
    }
}
