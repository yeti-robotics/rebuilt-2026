package frc.robot.subsystems.intake;

import static com.ctre.phoenix6.signals.MotorAlignmentValue.Aligned;
import static frc.robot.subsystems.intake.IntakeConfigs.PRIMARY_TALONFX_CONFIGS;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
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
        intakeMotor = new TalonFX(IntakeConfigs.PRIMARY_INTAKE_MOTOR_ID, Constants.rioBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(intakeMotor);
        }

        intakeMotor.getConfigurator().apply(PRIMARY_TALONFX_CONFIGS);
        intakeMotor.setControl(new Follower(IntakeConfigs.PRIMARY_INTAKE_MOTOR_ID, Aligned));
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
