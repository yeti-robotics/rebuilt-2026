package frc.robot.subsystems.shooter;

import static frc.robot.constants.Constants.currentMode;
import static frc.robot.subsystems.shooter.ShooterConfigsAlpha.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class ShooterIOGamma implements ShooterIO {
    public TalonFX firstMotor;
    public TalonFX secondMotor;
    public TalonFX thirdMotor;
    private final MotionMagicVelocityTorqueCurrentFOC MOTION_MAGIC_REQUEST = new MotionMagicVelocityTorqueCurrentFOC(0);

    private final DutyCycleOut dutyRequest = new DutyCycleOut(0.0);

    public ShooterIOGamma() {
        firstMotor = new TalonFX(ShooterConfigsGamma.FIRST_SHOOTER_ID, Constants.rioBus);
        secondMotor = new TalonFX(ShooterConfigsGamma.SECOND_SHOOTER_ID, Constants.rioBus);
        thirdMotor = new TalonFX(ShooterConfigsGamma.THIRD_SHOOTER_ID, Constants.rioBus);

        secondMotor.setControl(new Follower(ShooterConfigsGamma.FIRST_SHOOTER_ID, MotorAlignmentValue.Opposed));
        thirdMotor.setControl(new Follower(ShooterConfigsGamma.FIRST_SHOOTER_ID, MotorAlignmentValue.Aligned));

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(firstMotor);
            PhysicsSim.getInstance().addTalonFX(secondMotor);
            PhysicsSim.getInstance().addTalonFX(thirdMotor);
        }
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topMotorVoltage = firstMotor.getMotorVoltage().getValueAsDouble();
        inputs.topMotorRPM = firstMotor.getVelocity().getValueAsDouble();
        inputs.secondMotorVoltage = secondMotor.getMotorVoltage().getValueAsDouble();
        inputs.secondMotorRPM = secondMotor.getVelocity().getValueAsDouble();
        inputs.bottomMotorVoltage = thirdMotor.getMotorVoltage().getValueAsDouble();
        inputs.bottomMotorRPM = thirdMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void spinMotors(double velocity) {
        firstMotor.setControl(MOTION_MAGIC_REQUEST.withVelocity(velocity));
    }

    @Override
    public void stopMotors() {
        firstMotor.setVoltage(0);
    }

    @Override
    public void applyPower(double percent) {
        firstMotor.setControl(dutyRequest.withOutput(percent));
    }

    @Override
    public boolean isAtSpeed(double speed) {
        return firstMotor.getVelocity().isNear(20, 2);
    }
}