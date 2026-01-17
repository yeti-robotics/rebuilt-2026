package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConfigs.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class ShooterIOTalonFXAlpha implements ShooterIO {
    public TalonFX topMotor;
    public TalonFX bottomMotor;

    public ShooterIOTalonFXAlpha() {
        topMotor = new TalonFX(ShooterConfigs.TOP_MOTOR_ID, Constants.rioBus);
        bottomMotor = new TalonFX(ShooterConfigs.BOTTOM_MOTOR_ID, Constants.rioBus);
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
        inputs.bottomMotorVoltage = bottomMotor.getMotorVoltage().getValueAsDouble();
        inputs.bottomMotorRPM = bottomMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void spinMotors(double volts) {
        topMotor.setControl(new MotionMagicVelocityVoltage(volts));
    }

    @Override
    public void stopMotors() {
        topMotor.setVoltage(0);
    }
}
