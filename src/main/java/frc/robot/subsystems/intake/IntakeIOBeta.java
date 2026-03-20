package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class IntakeIOBeta implements IntakeIO {
    private final TalonFX rightIntakeMotor;
    private final TalonFX leftIntakeMotor;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public IntakeIOBeta() {
        rightIntakeMotor = new TalonFX(IntakeConfigsBeta.RIGHT_INTAKE_MOTOR_ID, Constants.rioBus);
        leftIntakeMotor = new TalonFX(IntakeConfigsBeta.LEFT_INTAKE_MOTOR_ID, Constants.rioBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(rightIntakeMotor);
            PhysicsSim.getInstance().addTalonFX(leftIntakeMotor);
        }

        rightIntakeMotor.getConfigurator().apply(IntakeConfigsBeta.RIGHT_TALONFX_CONFIGS);
        leftIntakeMotor.getConfigurator().apply(IntakeConfigsBeta.RIGHT_TALONFX_CONFIGS);
        leftIntakeMotor.setControl(new Follower(IntakeConfigsBeta.RIGHT_INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        inputs.primaryMotorRPM = rightIntakeMotor.getVelocity().getValueAsDouble();
        inputs.primaryMotorVoltage = rightIntakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.secondaryMotorRPM = leftIntakeMotor.getVelocity().getValueAsDouble();
        inputs.secondaryMotorVoltage = leftIntakeMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setIntakeMotor(double volts) {
        rightIntakeMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void applyPower(double percent) {
        rightIntakeMotor.setControl(dutyCycleOut.withOutput(percent));
    }
}
