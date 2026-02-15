package frc.robot.subsystems.linslide;

import static frc.robot.constants.Constants.rioBus;
import static frc.robot.subsystems.linslide.LinSlideConfigsAlpha.*;
import static frc.robot.subsystems.linslide.LinSlideConfigsBeta.LIN_SLIDE_CANCODER_ID;
import static frc.robot.subsystems.linslide.LinSlideConfigsBeta.linSlideCancoderConfiguration;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.util.sim.PhysicsSim;

public class LinSlideIOBeta implements LinSlideIO {
    public final TalonFX linSlideMotor;
    public final CANcoder linSlideCancoder;

    private final DutyCycleOut dutyRequest = new DutyCycleOut(0.0);

    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0);

    public LinSlideIOBeta() {
        linSlideMotor = new TalonFX(LIN_SLIDE_MOTOR_ID, rioBus);
        linSlideCancoder = new CANcoder(LIN_SLIDE_CANCODER_ID, rioBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(linSlideMotor);
        }
        linSlideMotor.getConfigurator().apply(linSlideTalonFXConfigs);
        linSlideCancoder.getConfigurator().apply(linSlideCancoderConfiguration);

        zeroPosition();
    }

    @Override
    public void updateInputs(LinSlideIO.LinSlideIOInputs inputs) {
        inputs.positionRotation = linSlideMotor.getPosition().getValueAsDouble();
        inputs.targetPositionRotation = linSlideMotor.getClosedLoopReference().getValueAsDouble();
        inputs.isDeployed = linSlideMotor.getPosition().getValueAsDouble() >= 3.1;
        inputs.isStowed = linSlideMotor.getPosition().getValueAsDouble() <= 0.1;
    }

    @Override
    public void zeroPosition() {
        linSlideMotor.setPosition(0);
    }

    @Override
    public void applyPower(double percent) {
        linSlideMotor.setControl(dutyRequest.withOutput(percent));
    }
}
