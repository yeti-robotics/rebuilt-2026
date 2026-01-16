package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.UpdateModeValue;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class ClimberIOAlpha implements ClimberIO {
    private final TalonFX climberMotor;
    private static CANrange climberSensor;

    public ClimberIOAlpha() {
        climberMotor = new TalonFX(ClimberConfig.climberMotorID, Constants.rioBus);
        climberSensor = new CANrange(ClimberConfig.climberSensorID, Constants.rioBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(climberMotor);
        }
        applyConfigs();
    }

    public static void applyConfigs() {
        // update these values for realzees
        var CANrangeConfig = new CANrangeConfiguration();
        CANrangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        CANrangeConfig.FovParams.FOVCenterX = 0; // Reset to default
        CANrangeConfig.FovParams.FOVCenterY = 0;
        CANrangeConfig.FovParams.FOVRangeX = 6.75; // Minimum
        CANrangeConfig.FovParams.FOVRangeY = 6.75; // Minimum
        CANrangeConfig.ProximityParams.ProximityThreshold = 0.10;
        CANrangeConfig.ProximityParams.ProximityHysteresis = 0.02;
        climberSensor.getConfigurator().apply(CANrangeConfig);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.position = climberMotor.getPosition().getValueAsDouble();
        inputs.isAtBottom = climberSensor.getIsDetected().getValue();
        inputs.targetPosition = climberMotor.getClosedLoopReference().getValueAsDouble();
    }



    @Override
    public void setClimberPosition(double position) {
        climberMotor.setControl(new MotionMagicTorqueCurrentFOC(position));
    }

    @Override
    public void zeroPosition() {climberMotor.setPosition(0);}

    @Override
    public void neutralizeClimber() {climberMotor.setControl(new NeutralOut());}

}
