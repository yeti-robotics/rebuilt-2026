package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class ShooterIOTalonFXAlpha implements ShooterIO {
    public TalonFX topMotor;
    public TalonFX bottomMotor;

    public ShooterIOTalonFXAlpha() {
        topMotor = new TalonFX(ShooterConfigs.topMotorID, Constants.canBus);
        bottomMotor = new TalonFX(ShooterConfigs.bottomMotorID, Constants.canBus);
        bottomMotor.setControl(new Follower(ShooterConfigs.topMotorID, MotorAlignmentValue.Opposed));

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(topMotor);
            PhysicsSim.getInstance().addTalonFX(bottomMotor);
        }
    }

    @Override
    public void updateInputs(ShooterIOAlphaInputs inputs) {
        inputs.topMotorVoltage = topMotor.getMotorVoltage().getValueAsDouble();
        inputs.topMotorRPM = topMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void rollMotors(double volts) {
        topMotor.setControl(new VoltageOut(volts));
    }
}
