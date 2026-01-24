package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConfigs.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class ShooterIOAlpha implements ShooterIO {
    public TalonFX topMotor;
    public TalonFX bottomMotor;
    private final MotionMagicVelocityVoltage MOTION_MAGIC_REQUEST = new MotionMagicVelocityVoltage(0);

    public ShooterIOAlpha() {
        topMotor = new TalonFX(ShooterConfigs.RIGHT_SHOOTER_ID, Constants.rioBus);
        bottomMotor = new TalonFX(ShooterConfigs.LEFT_SHOOTER_ID, Constants.rioBus);
        bottomMotor.setControl(new Follower(ShooterConfigs.RIGHT_SHOOTER_ID, MotorAlignmentValue.Opposed));
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
        topMotor.setControl(MOTION_MAGIC_REQUEST.withVelocity(volts));
    }

    @Override
    public void stopMotors() {
        topMotor.setVoltage(0);
    }
}
