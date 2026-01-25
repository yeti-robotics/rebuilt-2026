package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.intake.IntakeConfigs.INTAKE_TALONFX_CONFIGS;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.util.sim.PhysicsSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
    private final TalonFX intakeMotor;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final IntakeSimulation intakeSimulation;
    private final ShooterIOSim shooterIOSim;

    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain, ShooterIOSim shooterIOSim) {
        intakeMotor = new TalonFX(IntakeConfigs.INTAKE_MOTOR_ID, Constants.rioBus);
        PhysicsSim.getInstance().addTalonFX(intakeMotor);
        intakeMotor.getConfigurator().apply(INTAKE_TALONFX_CONFIGS);
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveTrain, // Drive simulation
                Inches.of(32.5), // Width of intake (placeholder value)
                Inches.of(12), // Extension length of intake (placeholder value)
                IntakeSimulation.IntakeSide.BACK, // Side intake is mounted on
                44); // placeholder value

        this.shooterIOSim = shooterIOSim;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerRPM = intakeMotor.getVelocity().getValueAsDouble();
        inputs.rollerVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
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
