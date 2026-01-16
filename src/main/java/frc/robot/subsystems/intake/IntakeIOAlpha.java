package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class IntakeIOAlpha implements IntakeIO {
    private final TalonFX intakeMotor;
    private final CANrange canRangeIntake;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public IntakeIOAlpha() {
        intakeMotor = new TalonFX(IntakeConfigs.INTAKE_MOTOR_ID, Constants.rioBus);
        canRangeIntake = new CANrange(IntakeConfigs.INTAKE_SENSOR_ID, Constants.rioBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(intakeMotor);
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerRPM = intakeMotor.getVelocity().getValueAsDouble();
        inputs.isDetected = canRangeIntake.getIsDetected().getValue();
        inputs.rollerVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setIntakeMotorDuty(double volts) {
        intakeMotor.setControl(voltageRequest.withOutput(volts));
    }
}
