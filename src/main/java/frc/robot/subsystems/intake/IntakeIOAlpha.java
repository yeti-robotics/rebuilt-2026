package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConfigsAlpha.ALPHA_TALONFX_CONFIGS;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class IntakeIOAlpha implements IntakeIO {
    private final TalonFX intakeMotor;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public IntakeIOAlpha() {
            intakeMotor = new TalonFX(IntakeConfigsAlpha.ALPHA_INTAKE_MOTOR_ID, Constants.rioBus);
            if (Robot.isSimulation()) {
                PhysicsSim.getInstance().addTalonFX(intakeMotor);
            }
            intakeMotor.getConfigurator().apply(IntakeConfigsAlpha.ALPHA_TALONFX_CONFIGS);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.primaryMotorRPM = intakeMotor.getVelocity().getValueAsDouble();
        inputs.primaryMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setIntakeMotor(double volts) {
        intakeMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void applyPower(double percent) {
        intakeMotor.setControl(dutyCycleOut.withOutput(percent));
    }
}
