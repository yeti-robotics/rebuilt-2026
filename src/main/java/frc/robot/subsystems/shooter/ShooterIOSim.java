package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final MotionMagicVelocityVoltage MOTION_MAGIC_REQUEST = new MotionMagicVelocityVoltage(0);
    public RebuiltFuelOnFly flyingFuel;
    private final Supplier<Pose2d> robotSimulationWorldPose;
    private final Supplier<ChassisSpeeds> chassisSpeedsFieldRelative;

    public ShooterIOSim(Supplier<Pose2d> robotSimulationWorldPose, Supplier<ChassisSpeeds> chassisSpeedsFieldRelative) {
        topMotor = new TalonFX(ShooterConfigs.RIGHT_SHOOTER_ID, Constants.rioBus);
        bottomMotor = new TalonFX(ShooterConfigs.LEFT_SHOOTER_ID, Constants.rioBus);
        topMotor.getConfigurator().apply(ShooterConfigs.TOP_MOTOR_CONFIGS);
        bottomMotor.getConfigurator().apply(ShooterConfigs.BOTTOM_MOTOR_CONFIGS);
        this.robotSimulationWorldPose = robotSimulationWorldPose;
        this.chassisSpeedsFieldRelative = chassisSpeedsFieldRelative;
        PhysicsSim.getInstance().addTalonFX(topMotor);
        PhysicsSim.getInstance().addTalonFX(bottomMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topMotorVoltage = topMotor.getMotorVoltage().getValueAsDouble();
        inputs.topMotorRPM = topMotor.getVelocity().getValueAsDouble();
        inputs.bottomMotorVoltage = topMotor.getMotorVoltage().getValueAsDouble();
        inputs.bottomMotorRPM = bottomMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void spinMotors(double volts) {
        topMotor.setControl(MOTION_MAGIC_REQUEST.withVelocity(volts));
    }

    @Override
    public void stopMotors() {
        topMotor.setVoltage(0);
    }

    @Override
    public void shootFuel() {
        flyingFuel = new RebuiltFuelOnFly(
                robotSimulationWorldPose.get().getTranslation(),
                new Translation2d(0.2, 0), // placeholder value
                chassisSpeedsFieldRelative.get(),
                robotSimulationWorldPose.get().getRotation().rotateBy(Rotation2d.k180deg),
                Meters.of(0.45), // placeholder value
                MetersPerSecond.of(8), // placeholder value
                Degrees.of(80) // placeholder value
                );

        flyingFuel
                .withTargetPosition(
                        () -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(0.25, 5.56, 2.3)))
                .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
                .withHitTargetCallBack(() -> System.out.println("Score: +1 point"))
                .enableBecomesGamePieceOnFieldAfterTouchGround();

        flyingFuel.withProjectileTrajectoryDisplayCallBack( // Shot Trajectory
                poses -> Logger.recordOutput("ShooterSim/ShotTrajectory", poses.toArray(Pose3d[]::new)));
        flyingFuel.enableBecomesGamePieceOnFieldAfterTouchGround();

        // Adding the gamepiece to the arena
        SimulatedArena.getInstance().addGamePieceProjectile(flyingFuel);
        Logger.recordOutput("ShooterSim/LastShotLaunched", true);
    }
}
