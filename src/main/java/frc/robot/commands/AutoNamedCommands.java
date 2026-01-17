package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;

import static com.pathplanner.lib.auto.NamedCommands.registerCommands;

public class AutoNamedCommands {
    private final IntakeSubsystem intake;
    private final Hopper hopper;



    public AutoNamedCommands(IntakeSubsystem intake, Hopper hopper) {
        this.intake = intake;
        this.hopper = hopper;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand("GroundIntake", Commands.sequence(
                hopper.spinHopper(0.0),
                intake.setRoller(0.0)
                ));
        NamedCommands.registerCommand("Shoot", Commands.sequence(
                hopper.spinHopper(5)
        ));
    }



}
