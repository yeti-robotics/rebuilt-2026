package frc.robot.subsystems.linslide;

import static frc.robot.constants.Constants.canBus;
import static frc.robot.subsystems.linslide.LinSlideConfigsAlpha.*;
import static frc.robot.subsystems.linslide.LinSlidePosition.DEPLOY;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import frc.robot.util.sim.PhysicsSim;

public class LinSlideIOAlpha implements LinearSlideIO {
    public final TalonFX LinSlideMotor;
    public final CANcoder LinSlideCANCoder;

    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0);

    public LinSlideIOAlpha() {
        LinSlideMotor = new TalonFX(LIN_SLIDE_MOTOR_ID, canBus);
        LinSlideCANCoder = new CANcoder(LIN_SLIDE_CANCODER_ID, canBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(LinSlideMotor);
        }
        LinSlideMotor.getConfigurator().apply(linSlideTalonFXConfigs);
        LinSlideCANCoder.getConfigurator().apply(linSlideCANCoderConfigs);
    }

    @Override
    public void updateInputs(LinearSlideIO.LinSlideIOInputs inputs) {
        inputs.positionRotation = LinSlideMotor.getPosition().getValueAsDouble();
        inputs.targetPositionRotation = LinSlideMotor.getClosedLoopReference().getValueAsDouble();
//      inputs.isDeployed = LinSlideCANCoder.getAbsolutePosition().isNear(DEPLOY, 0.1); // <-- Maybe
    }

    @Override
    public void moveToPosition(Angle position) {
        magicRequest.withPosition(position);
        LinSlideMotor.setControl(magicRequest);
    }
}
