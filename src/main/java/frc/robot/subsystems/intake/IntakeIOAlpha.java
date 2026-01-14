package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;

public class IntakeIOAlpha implements IntakeIO {
    public final TalonFX intakeMotor;
    public final CANrange canRangeIntake;

    public IntakeIOAlpha(){
        intakeMotor = new TalonFX(IntakeConfigs.intakeMotorID, Constants.canBus);
        canRangeIntake = new CANrange(IntakeConfigs.intakeSensorID, Constants.canBus);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.rollerRPM = intakeMotor.getVelocity().getValueAsDouble();
        inputs.isEngaged = canRangeIntake.getIsDetected().getValue();
        inputs.rollerVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setIntakeMotorDuty(double power) {
        intakeMotor.setControl(new DutyCycleOut(power));
    }
}
