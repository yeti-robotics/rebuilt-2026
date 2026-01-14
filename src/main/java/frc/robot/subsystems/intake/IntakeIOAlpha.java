package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class IntakeIOAlpha implements IntakeIO {
    public final TalonFX intakeMotor;
    public final CANrange canRangeIntake;

    public IntakeIOAlpha() {
        intakeMotor = new TalonFX(IntakeConfigs.intakeMotorID, Constants.canBus);
        canRangeIntake = new CANrange(IntakeConfigs.intakeCanRangeID, Constants.canBus);
        if (Robot.isSimulation()){
            PhysicsSim.getInstance().addTalonFX(intakeMotor);
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerRPM = intakeMotor.getVelocity().getValueAsDouble();
        inputs.isEngaged = canRangeIntake.getIsDetected().getValue();
        inputs.rollerVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setIntakeMotorDuty(double power) {
        intakeMotor.setControl(new DutyCycleOut(power));
    }
}
