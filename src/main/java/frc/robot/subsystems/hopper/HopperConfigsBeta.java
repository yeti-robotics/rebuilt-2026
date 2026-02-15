package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class HopperConfigsBeta {
    static final int BETA_ROLLER_ID = 60;
    static final int TOP_CANRANGE_ID = 23;
    static final int BOTTOM_CANRANGE_ID = 24;
    public static final double TEST_HOPPER_SPEED = -0.7;

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKG(0)
            .withKV(0)
            .withKA(0)
            .withKS(2)
            .withGravityType(GravityTypeValue.Elevator_Static);
    static final TalonFXConfiguration TalonFXConfigs = new TalonFXConfiguration()
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(2)
                    .withMotionMagicCruiseVelocity(4)
                    .withMotionMagicJerk(0));

    static final CANrangeConfiguration TOP_CANRANGE_CONFIGS = new CANrangeConfiguration()
            .withToFParams(new ToFParamsConfigs().withUpdateMode(UpdateModeValue.ShortRange100Hz))
            .withFovParams(new FovParamsConfigs()
                    .withFOVCenterX(0)
                    .withFOVCenterY(0)
                    .withFOVRangeX(27)
                    .withFOVRangeY(27))
            .withProximityParams(new ProximityParamsConfigs()
                    .withProximityThreshold(0.126)
                    .withProximityHysteresis(0.1)
                    .withMinSignalStrengthForValidMeasurement(2500));

    static final CANrangeConfiguration BOTTOM_CANRANGE_CONFIGS = new CANrangeConfiguration()
            .withToFParams(new ToFParamsConfigs().withUpdateMode(UpdateModeValue.ShortRange100Hz))
            .withFovParams(new FovParamsConfigs()
                    .withFOVCenterX(0)
                    .withFOVCenterY(0)
                    .withFOVRangeX(27)
                    .withFOVRangeY(27))
            .withProximityParams(new ProximityParamsConfigs()
                    .withProximityThreshold(0.126)
                    .withProximityHysteresis(0.1)
                    .withMinSignalStrengthForValidMeasurement(2500));

}
