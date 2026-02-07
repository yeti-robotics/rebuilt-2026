package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

import static com.ctre.phoenix6.signals.MotorAlignmentValue.Aligned;
import static frc.robot.subsystems.intake.IntakeConfigs.PRIMARY_TALONFX_CONFIGS;
import static frc.robot.subsystems.intake.IntakeConfigs.SECONDARY_TALONFX_CONFIGS;

public class IntakeIOBeta implements IntakeIO {
    private final TalonFX primaryIntakeMotor;
    private final TalonFX secondaryIntakeMotor;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public IntakeIOBeta(){
        primaryIntakeMotor = new TalonFX(IntakeConfigs.PRIMARY_INTAKE_MOTOR_ID, Constants.rioBus);
        secondaryIntakeMotor = new TalonFX(IntakeConfigs.SECONDARY_INTAKE_MOTOR_ID, Constants.rioBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(primaryIntakeMotor);
            PhysicsSim.getInstance().addTalonFX(secondaryIntakeMotor);
    }

        primaryIntakeMotor.getConfigurator().apply(PRIMARY_TALONFX_CONFIGS);
        secondaryIntakeMotor.getConfigurator().apply(SECONDARY_TALONFX_CONFIGS);
        primaryIntakeMotor.setControl(new Follower(IntakeConfigs.PRIMARY_INTAKE_MOTOR_ID, Aligned));
    }

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        inputs.primaryMotorRPM = primaryIntakeMotor.getVelocity().getValueAsDouble();
        inputs.primaryMotorVoltage = primaryIntakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.secondaryMotorRPM = secondaryIntakeMotor.getVelocity().getValueAsDouble();
        inputs.secondaryMotorVoltage = secondaryIntakeMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setIntakeMotor(double volts) {
        primaryIntakeMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void applyPower(double percent) {
        primaryIntakeMotor.setControl(dutyCycleOut.withOutput(percent));
    }
}
