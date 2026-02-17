package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.util.sim.PhysicsSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public class IntakeIOAlphaSim implements IntakeIO {
    private final TalonFX intakeMotor;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final IntakeSimulation intakeSimulation;
    private final ShooterIOSim shooterIOSim;

    public IntakeIOAlphaSim(AbstractDriveTrainSimulation driveTrain, ShooterIOSim shooterIOSim) {
        intakeMotor = new TalonFX(IntakeConfigsAlpha.ALPHA_INTAKE_MOTOR_ID, Constants.rioBus);
        PhysicsSim.getInstance().addTalonFX(intakeMotor);
        intakeMotor.getConfigurator().apply(IntakeConfigsAlpha.ALPHA_TALONFX_CONFIGS);
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveTrain, // Drive simulation
                Inches.of(24), // Width of intake
                Inches.of(7), // Extension length of intake
                IntakeSimulation.IntakeSide.FRONT, // Side intake is mounted on
                44); // placeholder value

        this.shooterIOSim = shooterIOSim;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.primaryMotorRPM = intakeMotor.getVelocity().getValueAsDouble();
        inputs.primaryMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        Logger.recordOutput("Intake run", intakeSimulation.isRunning());
        Logger.recordOutput("Intake balls", intakeSimulation.getGamePiecesAmount());
    }

    @Override
    public void setIntakeMotor(double volts) {
        intakeMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setRunning(boolean runIntake) {
        if (runIntake) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }
    }

    @Override
    public boolean isFuelInsideIntake(boolean isFuelInsideIntake) {
        return intakeSimulation.getGamePiecesAmount() != 0;
    }

    @Override
    public void handoffFuel() {
        if (intakeSimulation.obtainGamePieceFromIntake()) {
            shooterIOSim.shootFuel();
        }
    }
}
