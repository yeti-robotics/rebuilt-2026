package frc.robot.commands;

import static com.pathplanner.lib.auto.NamedCommands.registerCommands;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.FieldConstants.Hub.centerHubOpening;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.linslide.LinSlidePosition;
import frc.robot.subsystems.linslide.LinSlideSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoNamedCommands {
    private final IntakeSubsystem intake;
    private final Hopper hopper;
    private final ShooterSubsystem shooter;
    private final Climber climber;
    private final LinSlideSubsystem linSlide;
    private final Drive drive;
    private final CommandXboxController controller;

    public AutoNamedCommands(
            IntakeSubsystem intake,
            Hopper hopper,
            ShooterSubsystem shooter,
            Climber climber,
            LinSlideSubsystem linSlide,
            AutoAimCommands autoAim,
            Drive drive,
            CommandXboxController controller) {
        this.intake = intake;
        this.hopper = hopper;
        this.shooter = shooter;
        this.climber = climber;
        this.linSlide = linSlide;
        this.drive = drive;
        this.controller = controller;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand(
                "GroundIntakeOut",
                Commands.sequence(linSlide.moveToPosition(LinSlidePosition.DEPLOY.getPosition()), intake.rollIn()));
        NamedCommands.registerCommand(
                "GroundIntakeIn", Commands.sequence(linSlide.moveToPosition(LinSlidePosition.STOW.getPosition())));
        NamedCommands.registerCommand(
                "AutoAlignShoot",
                Commands.sequence(
                        AutoAimCommands.autoAim(
                                drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                centerHubOpening.toTranslation2d()),
                        hopper.spinHopper(0),
                        shooter.shoot(0)));
        NamedCommands.registerCommand("Climb", climber.moveToPosition(Rotations.of(0.0)));
    }
}
