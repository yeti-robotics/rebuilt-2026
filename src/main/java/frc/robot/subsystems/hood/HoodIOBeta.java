package frc.robot.subsystems.hood;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class HoodIOBeta implements HoodIO {
    private final TalonFX hoodMotor;
    private final CANcoder hoodCANcoder;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public HoodIOBeta() {
        hoodMotor = new TalonFX(HoodConfigsBeta.HOOD_MOTOR_ID, Constants.rioBus);
        hoodCANcoder = new CANcoder(HoodConfigsBeta.HOOD_CANCODER_ID, Constants.rioBus);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(hoodMotor, hoodCANcoder);
        }

        hoodMotor.getConfigurator().apply(HoodConfigsBeta.HOOD_MOTOR_CONFIGS);
        hoodCANcoder.getConfigurator().apply(HoodConfigsBeta.HOOD_CANCODER_CONFIGS);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodPosition = hoodMotor.getPosition().getValueAsDouble();
        inputs.hoodTargetPosition = hoodMotor.getClosedLoopReference().getValueAsDouble();
        inputs.hoodVelocity = hoodMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void moveToPosition(Angle position) {
        hoodMotor.setControl(positionRequest.withPosition(position));
    }

    @Override
    public void applyPower(double percent) {
        hoodMotor.setControl(dutyCycleOut.withOutput(percent));
    }
}
