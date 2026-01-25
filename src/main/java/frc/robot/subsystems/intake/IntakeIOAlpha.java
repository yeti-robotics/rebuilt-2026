package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConfigs.INTAKE_TALONFX_CONFIGS;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class IntakeIOAlpha implements IntakeIO {
    private final TalonFX intakeMotor;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public IntakeIOAlpha() {
        intakeMotor = new TalonFX(IntakeConfigs.INTAKE_MOTOR_ID, Constants.rioBus);
        intakeMotor.getConfigurator().apply(INTAKE_TALONFX_CONFIGS);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerRPM = intakeMotor.getVelocity().getValueAsDouble();
        inputs.rollerVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setIntakeMotor(double power) {
        intakeMotor.setControl(dutyCycleOut.withOutput(power));
    }
}
