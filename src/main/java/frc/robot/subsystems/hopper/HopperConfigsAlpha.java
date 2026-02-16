package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class HopperConfigsAlpha {
    static final int ROLLER_ID = 60;
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
}
