package frc.robot.subsystems.arm;

import static frc.robot.constants.Constants.canBus;
import static frc.robot.subsystems.arm.ArmConfigsAlpha.armMotorID;
import static frc.robot.subsystems.arm.ArmConfigsAlpha.armTalonFXConfigs;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import frc.robot.util.sim.PhysicsSim;

public class ArmIOTalonFXAlpha implements ArmIOAlpha {
    public final TalonFX armMotor;

    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0);

    public ArmIOTalonFXAlpha() {
        armMotor = new TalonFX(armMotorID, canBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(armMotor);
        }
        armMotor.getConfigurator().apply(armTalonFXConfigs);
    }

    @Override
    public void updateInputs(ArmIOAlpha.ArmIOAlphaInputs inputs) {
        inputs.positionRotation = armMotor.getPosition().getValueAsDouble();
        inputs.targetPositionRotation = armMotor.getClosedLoopReference().getValueAsDouble();
    }

    @Override
    public void moveToPosition(Angle position) {
        magicRequest.withPosition(position);
        armMotor.setControl(magicRequest);
    }
}
