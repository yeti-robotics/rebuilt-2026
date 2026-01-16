package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;
import static frc.robot.subsystems.shooter.ShooterConfigs.*;

public class ShooterIOTalonFXAlpha implements ShooterIO {
    public TalonFX topMotor;
    public TalonFX bottomMotor;

    public ShooterIOTalonFXAlpha() {
        topMotor = new TalonFX(ShooterConfigs.TOP_MOTOR_ID, Constants.canBus);
        bottomMotor = new TalonFX(ShooterConfigs.BOTTOM_MOTOR_ID, Constants.canBus);
        bottomMotor.setControl(new Follower(ShooterConfigs.TOP_MOTOR_ID, MotorAlignmentValue.Opposed));
        topMotor.getConfigurator().apply(TOP_MOTOR_CONFIGS);
        bottomMotor.getConfigurator().apply(BOTTOM_MOTOR_CONFIGS);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(topMotor);
            PhysicsSim.getInstance().addTalonFX(bottomMotor);
        }
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topMotorVoltage = topMotor.getMotorVoltage().getValueAsDouble();
        inputs.topMotorRPM = topMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void rollMotors(double volts) {
        topMotor.setControl(new VoltageOut(volts));
    }
}
