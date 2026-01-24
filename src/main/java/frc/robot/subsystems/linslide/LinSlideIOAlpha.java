package frc.robot.subsystems.linslide;

import static frc.robot.constants.Constants.rioBus;
import static frc.robot.subsystems.linslide.LinSlideConfigsAlpha.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import frc.robot.util.sim.PhysicsSim;

public class LinSlideIOAlpha implements LinSlideIO {
    public final TalonFX linSlideMotor;

    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0);

    public LinSlideIOAlpha() {
        linSlideMotor = new TalonFX(LIN_SLIDE_MOTOR_ID, rioBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(linSlideMotor);
        }
        linSlideMotor.getConfigurator().apply(linSlideTalonFXConfigs);

        zeroPosition();
    }

    @Override
    public void updateInputs(LinSlideIO.LinSlideIOInputs inputs) {
        inputs.positionRotation = linSlideMotor.getPosition().getValueAsDouble();
        inputs.targetPositionRotation = linSlideMotor.getClosedLoopReference().getValueAsDouble();
    }

    @Override
    public void moveToPosition(Angle position) {
        linSlideMotor.setControl(magicRequest.withPosition(position));
    }

    @Override
    public void applyPower(double percent) {
        linSlideMotor.setControl(new DutyCycleOut(percent));
    }
}
