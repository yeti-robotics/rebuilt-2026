package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class ShooterIOGamma implements ShooterIO {
    public TalonFX leaderMotor;
    public TalonFX followerMotor;
    public TalonFX followerMotor2;
    private final MotionMagicVelocityTorqueCurrentFOC MOTION_MAGIC_REQUEST = new MotionMagicVelocityTorqueCurrentFOC(0);

    private final DutyCycleOut dutyRequest = new DutyCycleOut(0.0);

    public ShooterIOGamma() {
        leaderMotor = new TalonFX(ShooterConfigsGamma.LEADER_SHOOTER_ID, Constants.rioBus);
        followerMotor = new TalonFX(ShooterConfigsGamma.FOLLOWER1_SHOOTER_ID, Constants.rioBus);
        followerMotor2 = new TalonFX(ShooterConfigsGamma.FOLLOWER2_SHOOTER_ID, Constants.rioBus);

        leaderMotor.getConfigurator().apply(ShooterConfigsGamma.LEADER_MOTOR_CONFIGS);
        followerMotor.getConfigurator().apply(ShooterConfigsGamma.EMPTY_SLOT_0);
        followerMotor2.getConfigurator().apply(ShooterConfigsGamma.EMPTY_SLOT_0);

        followerMotor.setControl(new Follower(ShooterConfigsGamma.LEADER_SHOOTER_ID, MotorAlignmentValue.Opposed));
        followerMotor2.setControl(new Follower(ShooterConfigsGamma.LEADER_SHOOTER_ID, MotorAlignmentValue.Opposed));

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(leaderMotor);
            PhysicsSim.getInstance().addTalonFX(followerMotor);
            PhysicsSim.getInstance().addTalonFX(followerMotor2);
        }
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topMotorVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
        inputs.topMotorRPM = leaderMotor.getVelocity().getValueAsDouble();
        inputs.secondMotorVoltage = followerMotor.getMotorVoltage().getValueAsDouble();
        inputs.secondMotorRPM = followerMotor.getVelocity().getValueAsDouble();
        inputs.bottomMotorVoltage = followerMotor2.getMotorVoltage().getValueAsDouble();
        inputs.bottomMotorRPM = followerMotor2.getVelocity().getValueAsDouble();
        inputs.closedLoopSlot = MOTION_MAGIC_REQUEST.Slot;
    }

    @Override
    public void spinMotors(double velocity) {
        leaderMotor.setControl(MOTION_MAGIC_REQUEST.withVelocity(velocity));
    }

    @Override
    public void stopMotors() {
        leaderMotor.setVoltage(0);
    }

    @Override
    public void applyPower(double percent) {
        leaderMotor.setControl(dutyRequest.withOutput(percent));
    }

    @Override
    public void switchSlot(int slotNum) {
        MOTION_MAGIC_REQUEST.Slot = slotNum;
    }

    @Override
    public boolean isAtSpeed(double speed) {
        return leaderMotor.getVelocity().isNear(speed, 2);
    }
}
