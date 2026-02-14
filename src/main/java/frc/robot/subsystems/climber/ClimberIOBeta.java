package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class ClimberIOBeta implements ClimberIO {
    private final TalonFX climberMotor;
    private final Servo linServo;
    private MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0);
    private DutyCycleOut dutyRequest = new DutyCycleOut(0);

    public ClimberIOBeta() {
        climberMotor = new TalonFX(ClimberConfig.CLIMBER_MOTOR_ID, Constants.rioBus);
        linServo = new Servo(ClimberConfig.LINEAR_SERVO_CHANNEL);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(climberMotor);
        }
        climberMotor.getConfigurator().apply(ClimberConfig.primaryTalonFXConfigs);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.position = climberMotor.getPosition().getValueAsDouble();
        inputs.targetPosition = climberMotor.getClosedLoopReference().getValueAsDouble();
    }

    @Override
    public void setClimberPosition(Angle position) {
        climberMotor.setControl(magicRequest.withPosition(position));
    }

    @Override
    public void zeroPosition() {
        climberMotor.setPosition(0);
    }

    @Override
    public void neutralizeClimber() {
        climberMotor.setControl(new NeutralOut());
    }

    @Override
    public void applyPower(double percent) {
        climberMotor.setControl(dutyRequest.withOutput(percent));
    }

    @Override
    public void setAngle(double position) {
        linServo.setAngle(position);
    }
}
