package frc.robot;

import static frc.robot.constants.FieldConstants.Hub.centerHubOpening;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAimCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberPosition;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.linslide.LinSlideSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.PathPlannerUtils;
import java.util.Optional;

public class AutoCommands {
    private final Climber climber;
    private final CommandSwerveDrivetrain drivetrain;
    private final HoodSubsystem hood;
    private final Hopper hopper;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final LinSlideSubsystem linSlide;
    private final ShooterSubsystem shooter;

    public AutoCommands(
            Climber climber,
            CommandSwerveDrivetrain drivetrain,
            HoodSubsystem hood,
            Hopper hopper,
            IndexerSubsystem indexer,
            IntakeSubsystem intake,
            LinSlideSubsystem linSlide,
            ShooterSubsystem shooter) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.hopper = hopper;
        this.indexer = indexer;
        this.intake = intake;
        this.linSlide = linSlide;
        this.shooter = shooter;
    }

    public Command shoot() {
        return Commands.sequence(
                AutoAimCommands.autoAim(
                        drivetrain,
                        () -> 0,
                        () -> 0,
                        centerHubOpening.toTranslation2d())
                        .withTimeout(1),
                indexer.runMotors(0),
                shooter.shoot(0),
                hopper.spinHopper(0));
    }

    public Command intake() {
        return Commands.sequence(linSlide.moveToPosition(0.4, true), intake.rollIn());
    }

    public Command oneCycleNeutralTowerLeft() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_L-left");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-left");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-left");

        PathPlannerAuto auto;

        var cmd = startNeutral.isEmpty() || neutralShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none().andThen(Commands.print("Command is Empty"))
                : Commands.sequence(
                        shoot().withTimeout(2),
                        AutoBuilder.followPath(startNeutral.get())
                                .alongWith(Commands.waitSeconds(4))
                                .andThen(intake())
                                .withTimeout(4),
                        AutoBuilder.followPath(neutralShoot.get()).alongWith(linSlide.moveToPosition(-0.4, false)),
                        shoot().withTimeout(2),
                        AutoBuilder.followPath(shootTower.get()),
                        climber.moveToPosition(ClimberPosition.L1.getHeight()));

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command oneCycleDepotTowerLeft() {
        Optional<PathPlannerPath> startDepot = PathPlannerUtils.loadPathByName("start-depot-left");
        Optional<PathPlannerPath> depotShoot = PathPlannerUtils.loadPathByName("depot-shoot-left");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-left");

        PathPlannerAuto auto;

        var cmd = startDepot.isEmpty() || depotShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none().andThen(Commands.print("Command is Empty"))
                : Commands.sequence(
                Commands.print("Command is working"),
                shoot().withTimeout(2),
                AutoBuilder.followPath(startDepot.get())
                        .alongWith(Commands.waitSeconds(1))
                        .andThen(intake())
                        .withTimeout(4),
                AutoBuilder.followPath(depotShoot.get()).alongWith(linSlide.moveToPosition(-0.4, false)),
                shoot().withTimeout(2),
                AutoBuilder.followPath(shootTower.get()),
                climber.moveToPosition(ClimberPosition.L1.getHeight()));

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command oneCycleNeutralLeftTowerCenter() {
        Optional<PathPlannerPath> initNeutral = PathPlannerUtils.loadPathByName("init-neutral_L-center");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-center");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower_L-center");

        PathPlannerAuto auto;

        var cmd = initNeutral.isEmpty() || neutralShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        shoot().withTimeout(2),
                        AutoBuilder.followPath(initNeutral.get())
                                .alongWith(Commands.waitSeconds(4))
                                .andThen(intake().withTimeout(2)),
                        AutoBuilder.followPath(neutralShoot.get()),
                        shoot().withTimeout(2),
                        Commands.waitSeconds(2).andThen(linSlide.moveToPosition(-0.4, false)),
                        AutoBuilder.followPath(shootTower.get()));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command oneCycleNeutralRightTowerRight() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_R-right");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_R-shoot-right");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-right");

        PathPlannerAuto auto;

        var cmd = startNeutral.isEmpty() || neutralShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        shoot().withTimeout(2),
                        AutoBuilder.followPath(startNeutral.get())
                                .alongWith(Commands.waitSeconds(4))
                                .andThen(intake().withTimeout(2)),
                        AutoBuilder.followPath(neutralShoot.get()),
                        shoot().withTimeout(2),
                        Commands.waitSeconds(2).andThen(linSlide.moveToPosition(-0.4, false)),
                        AutoBuilder.followPath(shootTower.get()));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }
}