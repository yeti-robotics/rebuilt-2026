package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class ClimberIOAlpha implements ClimberIO {
    private final TalonFX climberMotor;
    private static CANrange climberSensor;

    public ClimberIOAlpha() {
        climberMotor = new TalonFX(ClimberConfig.CLIMBER_MOTOR_ID, Constants.rioBus);
        climberSensor = new CANrange(ClimberConfig.CLIMBER_SENSOR_ID, Constants.rioBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(climberMotor);
        }
        climberMotor.getConfigurator().apply(ClimberConfig.primaryTalonFXConfigs);
        applyConfigs();
    }

    public static void applyConfigs() {
        // update these values for realzees
        // var DigitalInputConfig = new DigitalInputsConfigs();
        // DigitalInputConfig.
        // climberSensor.getConfigurator().apply(CANrangeConfig);

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.position = climberMotor.getPosition().getValueAsDouble();
        // for a canrange
        inputs.isAtBottom = climberSensor.getIsDetected().getValue();
        // for a limit switch
        //inputs.isAtBottom = climberSensor.get();
        inputs.targetPosition = climberMotor.getClosedLoopReference().getValueAsDouble();
    }

    @Override
    public void setClimberPosition(Angle position) {
        climberMotor.setControl(new MotionMagicTorqueCurrentFOC(position));
    }

    @Override
    public void zeroPosition() {
        climberMotor.setPosition(0);
    }

    @Override
    public void neutralizeClimber() {
        climberMotor.setControl(new NeutralOut());
    }
}
