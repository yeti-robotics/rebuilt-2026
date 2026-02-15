package frc.robot.subsystems.shooter;

import static frc.robot.constants.Constants.currentMode;
import static frc.robot.subsystems.shooter.ShooterConfigsAlpha.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class ShooterIOReal implements ShooterIO {
    public TalonFX topMotor;
    public TalonFX bottomMotor;
    private final MotionMagicVelocityTorqueCurrentFOC MOTION_MAGIC_REQUEST = new MotionMagicVelocityTorqueCurrentFOC(0);

    private final DutyCycleOut dutyRequest = new DutyCycleOut(0.0);

    public ShooterIOReal() {
        if (currentMode == Constants.Mode.ALPHA) {
            topMotor = new TalonFX(ShooterConfigsAlpha.RIGHT_SHOOTER_ID, Constants.rioBus);
            bottomMotor = new TalonFX(ShooterConfigsAlpha.LEFT_SHOOTER_ID, Constants.rioBus);
            bottomMotor.setControl(new Follower(ShooterConfigsAlpha.RIGHT_SHOOTER_ID, MotorAlignmentValue.Opposed));
            topMotor.getConfigurator().apply(ShooterConfigsAlpha.TOP_MOTOR_CONFIGS);
            bottomMotor.getConfigurator().apply(ShooterConfigsAlpha.BOTTOM_MOTOR_CONFIGS);

            if (Robot.isSimulation()) {
                PhysicsSim.getInstance().addTalonFX(topMotor);
                PhysicsSim.getInstance().addTalonFX(bottomMotor);
            }
        } else {
            topMotor = new TalonFX(ShooterConfigsBeta.RIGHT_SHOOTER_ID, Constants.rioBus);
            bottomMotor = new TalonFX(ShooterConfigsBeta.LEFT_SHOOTER_ID, Constants.rioBus);
            bottomMotor.setControl(new Follower(ShooterConfigsBeta.RIGHT_SHOOTER_ID, MotorAlignmentValue.Opposed));
            topMotor.getConfigurator().apply(ShooterConfigsBeta.TOP_MOTOR_CONFIGS);
            bottomMotor.getConfigurator().apply(ShooterConfigsBeta.BOTTOM_MOTOR_CONFIGS);

            if (Robot.isSimulation()) {
                PhysicsSim.getInstance().addTalonFX(topMotor);
                PhysicsSim.getInstance().addTalonFX(bottomMotor);
            }
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
    public void spinMotors(double velocity) {
        topMotor.setControl(MOTION_MAGIC_REQUEST.withVelocity(velocity));
    }

    @Override
    public void stopMotors() {
        topMotor.setVoltage(0);
    }

    @Override
    public void applyPower(double percent) {
        topMotor.setControl(dutyRequest.withOutput(percent));
    }

    @Override
    public boolean isAtSpeed(double speed) {
        return topMotor.getVelocity().isNear(20, 2);
    }
}
