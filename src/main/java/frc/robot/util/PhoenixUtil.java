// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }

    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        private static int instances = 0;
        public final int id;

        private final TalonFXSimState talonFXSimState;

        public TalonFXMotorControllerSim(TalonFX talonFX) {
            this.id = instances++;

            this.talonFXSimState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    public static class TalonFXMotorControllerWithRemoteCancoderSim extends TalonFXMotorControllerSim {
        private final CANcoderSimState remoteCancoderSimState;

        public TalonFXMotorControllerWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
            super(talonFX);
            this.remoteCancoderSimState = cancoder.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            remoteCancoderSimState.setRawPosition(mechanismAngle);
            remoteCancoderSimState.setVelocity(mechanismVelocity);

            return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }

    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] = Timer.getFPGATimestamp()
                    - 0.02
                    + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }

    public static SwerveModuleConstants regulateModuleConstantForSimulation(
            SwerveModuleConstants<?, ?, ?> moduleConstants) {
        // Skip regulation if running on a real robot
        if (RobotBase.isReal()) return moduleConstants;

        // Apply simulation-specific adjustments to module constants
        return moduleConstants
                // Disable encoder offsets
                .withEncoderOffset(0)
                // Disable motor inversions for drive and steer motors
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                // Disable CanCoder inversion
                .withEncoderInverted(false)
                // Adjust steer motor PID gains for simulation
                .withSteerMotorGains(new Slot0Configs()
                        .withKP(70)
                        .withKI(0)
                        .withKD(4.5)
                        .withKS(0)
                        .withKV(1.91)
                        .withKA(0)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withSteerMotorGearRatio(16.0)
                // Adjust friction voltages
                .withDriveFrictionVoltage(Volts.of(0.1))
                .withSteerFrictionVoltage(Volts.of(0.05))
                // Adjust steer inertia
                .withSteerInertia(KilogramSquareMeters.of(0.05));
    }
}
