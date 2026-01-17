package frc.robot.commands;

import static com.pathplanner.lib.auto.NamedCommands.registerCommands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoNamedCommands {
    private final IntakeSubsystem intake;
    private final Hopper hopper;
    private final ShooterSubsystem shooter;
    private final Climber climber;

    public AutoNamedCommands(IntakeSubsystem intake, Hopper hopper, ShooterSubsystem shooter, Climber climber) {
        this.intake = intake;
        this.hopper = hopper;
        this.shooter = shooter;
        this.climber = climber;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand("GroundIntake", Commands.sequence(hopper.spinHopper(0.0),
                intake.setRoller(0.0)));
        NamedCommands.registerCommand("Shoot", Commands.sequence(hopper.spinHopper(0),
                shooter.shoot(0)));
    }
}
