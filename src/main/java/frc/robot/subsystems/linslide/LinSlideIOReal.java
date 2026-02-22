package frc.robot.subsystems.linslide;

import static frc.robot.constants.Constants.currentMode;
import static frc.robot.constants.Constants.rioBus;
import static frc.robot.subsystems.linslide.LinSlideConfigsAlpha.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class LinSlideIOReal implements LinSlideIO {
    public final TalonFX linSlideMotor;
    private CANcoder linSlideCANcoder;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0);

    public LinSlideIOReal() {
        if (currentMode == Constants.Mode.ALPHA) {
            linSlideMotor = new TalonFX(LinSlideConfigsAlpha.LIN_SLIDE_MOTOR_ID, rioBus);
            if (Robot.isSimulation()) {
                PhysicsSim.getInstance().addTalonFX(linSlideMotor);
            }
            linSlideMotor.getConfigurator().apply(LinSlideConfigsAlpha.linSlideTalonFXConfigs);
            zeroPosition();

        } else {
            linSlideMotor = new TalonFX(LinSlideConfigsBeta.LIN_SLIDE_MOTOR_ID, rioBus);
            linSlideCANcoder = new CANcoder(LinSlideConfigsBeta.LIN_SLIDE_CANCODER_ID, rioBus);
            if (Robot.isSimulation()) {
                PhysicsSim.getInstance().addTalonFX(linSlideMotor, linSlideCANcoder);
            }
            linSlideMotor.getConfigurator().apply(LinSlideConfigsBeta.linSlideTalonFXConfigs);
            linSlideCANcoder.getConfigurator().apply(LinSlideConfigsBeta.linSlideCancoderConfiguration);
            zeroPosition();
        }
    }

    @Override
    public void updateInputs(LinSlideIO.LinSlideIOInputs inputs) {
        inputs.positionRotation = linSlideMotor.getPosition().getValueAsDouble();
        inputs.targetPositionRotation = linSlideMotor.getClosedLoopReference().getValueAsDouble();
        inputs.isDeployed = linSlideMotor.getPosition().getValueAsDouble() >= 3.1;
        inputs.isStowed = linSlideMotor.getPosition().getValueAsDouble() <= 0.1;
        inputs.velocityRPM = linSlideMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void zeroPosition() {
        linSlideMotor.setPosition(0);
    }

    @Override
    public void applyPower(double percent) {
        linSlideMotor.setControl(dutyRequest.withOutput(percent));
    }

    @Override
    public void defaultCommand(double volts) {
        linSlideMotor.setControl(voltageRequest.withOutput(volts));
    }
}
