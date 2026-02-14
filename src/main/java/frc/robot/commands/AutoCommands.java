package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.constants.FieldConstants.Hub.centerHubOpening;
import static frc.robot.subsystems.hopper.HopperConfigs.TEST_HOPPER_SPEED;
import static frc.robot.subsystems.indexer.IndexerConfigs.TEST_INDEXER_SPEED;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import org.littletonrobotics.junction.Logger;

public class AutoCommands {
    private final Climber climber;
    private final CommandSwerveDrivetrain drivetrain;
    private final HoodSubsystem hood;
    private final Hopper hopper;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final LinSlideSubsystem linSlide;
    private final ShooterSubsystem shooter;

    // public final Trigger indexerTrigger;

    public AutoCommands(
            Climber climber,
            CommandSwerveDrivetrain drivetrain,
            HoodSubsystem hood,
            Hopper hopperAuto,
            IndexerSubsystem indexerAuto,
            IntakeSubsystem intake,
            LinSlideSubsystem linSlide,
            ShooterSubsystem shooter) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.hopper = hopperAuto;
        this.indexer = indexerAuto;
        this.intake = intake;
        this.linSlide = linSlide;
        this.shooter = shooter;

        //        indexerTrigger = new Trigger(() -> !indexer.canRangeDetected())
        //                .and(() -> shooter.getTargetSpeed() > 0)
        //                .debounce(4)
        //                .onTrue(shooter.stopFlywheels());
    }

    public Command aimAndRev() {
        return Commands.sequence(
                        AutoAimCommands.autoAim(drivetrain, () -> 0, () -> 0, centerHubOpening.toTranslation2d())
                                .withTimeout(1),
                        shooter.revUpFlywheels(20).until(shooter::isAtSpeed))
                .andThen(() -> Logger.recordOutput("AutoTest", "Aimed and revved"));
    }

    public Command runFlywheels() {
        return shooter.shoot(20).andThen(() -> Logger.recordOutput("AutoTest", "Ran flywheels"));
    }

    public Command index() {
        return runOnce(() -> Logger.recordOutput("AutoTest", "Indexed and hopper"))
                .andThen(indexer.apply(0.5))
                .alongWith(hopper.apply(-0.7));
    }

    public Command stopShooting() {
        return runOnce(() -> shooter.stopFlywheels().alongWith(indexer.stop()).alongWith(hopper.stop()))
                .andThen(() -> Logger.recordOutput("AutoTest", "Stopped"));
    }

    public Command shoot() {
        return aimAndRev()
                .andThen(runFlywheels()
                        .alongWith(Commands.parallel(
                                hopper.applyPower(TEST_HOPPER_SPEED), indexer.applyPower(TEST_INDEXER_SPEED))))
                .withTimeout(3)
                .andThen(stopShooting());
    }

    public Command intake() {
        return Commands.sequence(linSlide.moveToPosition(0.5, true), intake.applyPower(-0.7));
    }

    public Command cycleNeutralRight(Optional<PathPlannerPath> pathOne, Optional<PathPlannerPath> pathTwo) {
        return Commands.sequence(
                AutoBuilder.followPath(pathOne.get())
                        .alongWith((Commands.waitSeconds(4)).andThen(intake().withTimeout(2))),
                AutoBuilder.followPath(pathTwo.get()),
                aimAndRev()
                        .withTimeout(2)
                        .alongWith(Commands.waitSeconds(2).andThen(linSlide.moveToPosition(-0.4, false))));
    }

    public Command followPathAndIntake(Optional<PathPlannerPath> path, int waitTime) {
        return AutoBuilder.followPath(path.get())
                .alongWith(Commands.waitSeconds(waitTime).andThen(intake().withTimeout(3)))
                .andThen(() -> Logger.recordOutput("AutoTest", "Followed path and intaked"));
    }

    public Command followPathAndStowIntake(Optional<PathPlannerPath> path) {
        return AutoBuilder.followPath(path.get())
                .alongWith(linSlide.moveToPosition(-0.2, false))
                .withTimeout(3);
    }

    public Command climbTower(Optional<PathPlannerPath> path) {
        return Commands.sequence(
                AutoBuilder.followPath(path.get()), climber.moveToPosition(ClimberPosition.L1.getHeight()));
    }

    // NEW STUFF

    public Command shootNew() {
        return Commands.sequence(
                AutoAimCommands.autoAim(drivetrain, () -> 0, () -> 0, centerHubOpening.toTranslation2d())
                        .withTimeout(1),
                shooter.revUpFlywheels(20).until(shooter::isAtSpeed),
                Commands.parallel(
                        shooter.shoot(20).withTimeout(2),
                        hopper.applyPower(TEST_HOPPER_SPEED).withTimeout(2),
                        indexer.applyPower(TEST_INDEXER_SPEED).withTimeout(2)));
    }

    public Command shootBumpFire() {
        return Commands.sequence(
                shooter.revUpFlywheels(20).until(shooter::isAtSpeed),
                Commands.parallel(
                        shooter.shoot(20).withTimeout(2),
                        hopper.applyPower(TEST_HOPPER_SPEED).withTimeout(2),
                        indexer.applyPower(TEST_INDEXER_SPEED).withTimeout(2)));
    }

    // Real Autos
    public Command oneCycleNeutralTowerLeft() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_L-left");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-left");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-left");

        PathPlannerAuto auto;

        var cmd = startNeutral.isEmpty() || neutralShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        shootNew(),
                        followPathAndIntake(startNeutral, 2),
                        followPathAndStowIntake(neutralShoot),
                        shootNew(),
                        climbTower(shootTower));

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command oneCycleDepotTowerLeft() {
        Optional<PathPlannerPath> startDepot = PathPlannerUtils.loadPathByName("start-depot-left");
        Optional<PathPlannerPath> depotShoot = PathPlannerUtils.loadPathByName("depot-shoot-left");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-left");

        PathPlannerAuto auto;

        var cmd = startDepot.isEmpty() || depotShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        shootNew(),
                        followPathAndIntake(startDepot, 4),
                        followPathAndStowIntake(depotShoot),
                        linSlide.moveToPosition(0.5, true),
                        shootNew(),
                        climbTower(shootTower));

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleNeutralDepotTowerLeft() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_L-left");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-left");
        Optional<PathPlannerPath> shootDepot = PathPlannerUtils.loadPathByName("shoot-depot-left");
        Optional<PathPlannerPath> depotShoot = PathPlannerUtils.loadPathByName("depot-shoot-left");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-left");

        PathPlannerAuto auto;

        var cmd = shootDepot.isEmpty()
                        || depotShoot.isEmpty()
                        || shootTower.isEmpty()
                        || startNeutral.isEmpty()
                        || neutralShoot.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        shootNew(),
                        followPathAndIntake(startNeutral, 4),
                        followPathAndStowIntake(neutralShoot),
                        shootNew(),
                        followPathAndIntake(shootDepot, 2),
                        followPathAndStowIntake(depotShoot),
                        shootNew(),
                        AutoBuilder.followPath(shootTower.get()),
                        climber.moveToPosition(ClimberPosition.L1.getHeight()));

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleDepotNeutralTowerLeft() {
        Optional<PathPlannerPath> shootNeutral = PathPlannerUtils.loadPathByName("shoot-neutral_L-left");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-left");
        Optional<PathPlannerPath> startDepot = PathPlannerUtils.loadPathByName("start-depot-left");
        Optional<PathPlannerPath> depotShoot = PathPlannerUtils.loadPathByName("depot-shoot-left");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-left");

        PathPlannerAuto auto;

        var cmd = startDepot.isEmpty()
                        || depotShoot.isEmpty()
                        || shootTower.isEmpty()
                        || shootNeutral.isEmpty()
                        || neutralShoot.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        shootNew(),
                        followPathAndIntake(startDepot, 2),
                        followPathAndStowIntake(depotShoot),
                        shootNew(),
                        followPathAndIntake(shootNeutral, 4),
                        followPathAndStowIntake(neutralShoot),
                        shootNew(),
                        climbTower(shootTower));

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleNeutralTowerLeft() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_L-left");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-left");
        Optional<PathPlannerPath> shootNeutralFarther = PathPlannerUtils.loadPathByName("shoot-neutral_L-farther-left");
        Optional<PathPlannerPath> neutralShootFarther = PathPlannerUtils.loadPathByName("neutral_L-shoot-farther-left");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-left");
        Optional<PathPlannerPath> shootNeutral = PathPlannerUtils.loadPathByName("shoot-neutral_L-left");

        PathPlannerAuto auto;

        var cmd = startNeutral.isEmpty()
                        || neutralShoot.isEmpty()
                        || shootTower.isEmpty()
                        || shootNeutralFarther.isEmpty()
                        || neutralShootFarther.isEmpty()
                ? Commands.none().andThen(Commands.print("Command is Empty"))
                : Commands.sequence(
                        shootNew(),
                        followPathAndIntake(startNeutral, 3),
                        followPathAndStowIntake(neutralShoot),
                        shootNew(),
                        followPathAndIntake(shootNeutralFarther, 3),
                        followPathAndStowIntake(neutralShootFarther),
                        shootNew(),
                        climbTower(shootTower));

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command oneCycleNeutralLeftTowerCenter() {
        Optional<PathPlannerPath> initNeutralL = PathPlannerUtils.loadPathByName("init-neutral_L-bump-center");
        Optional<PathPlannerPath> neutralLShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-center");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower_L-center");

        PathPlannerAuto auto;

        var cmd = initNeutralL.isEmpty() || neutralLShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        shootBumpFire(),
                        followPathAndIntake(initNeutralL, 2),
                        followPathAndStowIntake(neutralLShoot),
                        shootNew(),
                        climbTower(shootTower));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command oneCycleNeutralRightTowerCenter() {
        Optional<PathPlannerPath> initNeutralR = PathPlannerUtils.loadPathByName("init-neutral_R-center");
        Optional<PathPlannerPath> neutralRShoot = PathPlannerUtils.loadPathByName("neutral_R-shoot-center");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower_L-center");

        PathPlannerAuto auto;

        var cmd = initNeutralR.isEmpty() || neutralRShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        aimAndRev().withTimeout(2),
                        followPathAndIntake(initNeutralR, 4),
                        AutoBuilder.followPath(neutralRShoot.get()),
                        aimAndRev()
                                .withTimeout(2)
                                .alongWith(Commands.waitSeconds(2).andThen(linSlide.moveToPosition(-0.4, false))),
                        AutoBuilder.followPath(shootTower.get()));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleDepotNeutralLeftCenter() {
        Optional<PathPlannerPath> initDepot = PathPlannerUtils.loadPathByName("init-depot-center");
        Optional<PathPlannerPath> depotShoot = PathPlannerUtils.loadPathByName("depot-shoot-center");
        Optional<PathPlannerPath> shootNeutralL = PathPlannerUtils.loadPathByName("shoot-neutral_L-center");
        Optional<PathPlannerPath> neutralLShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-center");

        PathPlannerAuto auto;

        var cmd = initDepot.isEmpty() || depotShoot.isEmpty() || shootNeutralL.isEmpty() || neutralLShoot.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(initDepot, 2),
                        AutoBuilder.followPath(depotShoot.get())
                                .andThen(aimAndRev().andThen(linSlide.moveToPosition(-0.4, false))),
                        followPathAndIntake(shootNeutralL, 4),
                        AutoBuilder.followPath(neutralLShoot.get()),
                        aimAndRev()
                                .withTimeout(2)
                                .alongWith(Commands.waitSeconds(2).andThen(linSlide.moveToPosition(-0.4, false))));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleDepotNeutralRightCenter() {
        Optional<PathPlannerPath> initDepot = PathPlannerUtils.loadPathByName("init-depot-center");
        Optional<PathPlannerPath> depotShoot = PathPlannerUtils.loadPathByName("depot-shoot-center");
        Optional<PathPlannerPath> shootNeutralR = PathPlannerUtils.loadPathByName("shoot-neutral_R-center");
        Optional<PathPlannerPath> neutralRShoot = PathPlannerUtils.loadPathByName("neutral_R-shoot-center");

        PathPlannerAuto auto;

        var cmd = initDepot.isEmpty() || depotShoot.isEmpty() || shootNeutralR.isEmpty() || neutralRShoot.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(initDepot, 2),
                        AutoBuilder.followPath(depotShoot.get())
                                .andThen(aimAndRev().andThen(linSlide.moveToPosition(-0.4, false))),
                        followPathAndIntake(shootNeutralR, 4),
                        AutoBuilder.followPath(neutralRShoot.get()),
                        aimAndRev()
                                .withTimeout(2)
                                .alongWith(Commands.waitSeconds(2).andThen(linSlide.moveToPosition(-0.4, false))));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleNeutralNeutralLeftCenter() {
        Optional<PathPlannerPath> initNeutralL = PathPlannerUtils.loadPathByName("init-neutral_L-center");
        Optional<PathPlannerPath> neutralLShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-center");
        Optional<PathPlannerPath> shootNeutralL = PathPlannerUtils.loadPathByName("shoot-neutral_L-center");

        PathPlannerAuto auto;

        var cmd = initNeutralL.isEmpty() || neutralLShoot.isEmpty() || shootNeutralL.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        aimAndRev().withTimeout(2),
                        followPathAndIntake(initNeutralL, 4),
                        AutoBuilder.followPath(neutralLShoot.get()),
                        aimAndRev()
                                .withTimeout(2)
                                .alongWith(Commands.waitSeconds(2).andThen(linSlide.moveToPosition(-0.4, false))),
                        followPathAndIntake(shootNeutralL, 4),
                        AutoBuilder.followPath(neutralLShoot.get()),
                        aimAndRev()
                                .withTimeout(2)
                                .alongWith(Commands.waitSeconds(2).andThen(linSlide.moveToPosition(-0.4, false))));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleNeutralNeutralRightCenter() {
        Optional<PathPlannerPath> initNeutralR = PathPlannerUtils.loadPathByName("init-neutral_R-enter");
        Optional<PathPlannerPath> neutralRShoot = PathPlannerUtils.loadPathByName("neutral_R-shoot-center");
        Optional<PathPlannerPath> shootNeutralR = PathPlannerUtils.loadPathByName("shoot-neutral_R-center");

        PathPlannerAuto auto;

        var cmd = initNeutralR.isEmpty() || neutralRShoot.isEmpty() || shootNeutralR.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        aimAndRev().withTimeout(2),
                        cycleNeutralRight(initNeutralR, neutralRShoot),
                        cycleNeutralRight(shootNeutralR, neutralRShoot));
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
                        aimAndRev().withTimeout(2),
                        followPathAndIntake(startNeutral, 4),
                        followPathAndStowIntake(neutralShoot).andThen(aimAndRev()),
                        climbTower(shootTower));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command oneCycleOutpostTowerRight() {
        Optional<PathPlannerPath> startOutpost = PathPlannerUtils.loadPathByName("start-outpost-right");
        Optional<PathPlannerPath> outpostShoot = PathPlannerUtils.loadPathByName("outpost-shoot-right");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-right");

        PathPlannerAuto auto;

        var cmd = startOutpost.isEmpty() || outpostShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        shootNew(),
                        linSlide.moveToPosition(0.5, true),
                        AutoBuilder.followPath(startOutpost.get()),
                        (Commands.waitSeconds(3)),
                        AutoBuilder.followPath(outpostShoot.get()),
                        shootNew(),
                        linSlide.moveToPosition(-0.5, false),
                        climbTower(shootTower));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleOutpostNeutralTowerRight() {
        Optional<PathPlannerPath> startOutpost = PathPlannerUtils.loadPathByName("start-outpost-right");
        Optional<PathPlannerPath> outpostShoot = PathPlannerUtils.loadPathByName("outpost-shoot-right");
        Optional<PathPlannerPath> shootNeutral = PathPlannerUtils.loadPathByName("outpost-neutralZone-right");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_R-shoot-right");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-right");

        PathPlannerAuto auto;

        var cmd = startOutpost.isEmpty()
                        || outpostShoot.isEmpty()
                        || shootNeutral.isEmpty()
                        || neutralShoot.isEmpty()
                        || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        aimAndRev().withTimeout(2),
                        AutoBuilder.followPath(startOutpost.get()),
                        (Commands.waitSeconds(3)),
                        AutoBuilder.followPath(outpostShoot.get()),
                        aimAndRev().withTimeout(2),
                        followPathAndIntake(shootNeutral, 4),
                        followPathAndStowIntake(neutralShoot)
                                .andThen(aimAndRev().withTimeout(2)),
                        climbTower(shootTower));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleNeutralOutpostTowerRight() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_R-right");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_R-shoot-right");
        Optional<PathPlannerPath> shootOutpost = PathPlannerUtils.loadPathByName("shoot-outpost-right");
        Optional<PathPlannerPath> outpostShoot = PathPlannerUtils.loadPathByName("outpost-shoot-right");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-right");

        PathPlannerAuto auto;

        var cmd = startNeutral.isEmpty()
                        || neutralShoot.isEmpty()
                        || shootOutpost.isEmpty()
                        || outpostShoot.isEmpty()
                        || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        aimAndRev().withTimeout(2),
                        followPathAndIntake(startNeutral, 4),
                        followPathAndStowIntake(neutralShoot)
                                .andThen(aimAndRev().withTimeout(2)),
                        AutoBuilder.followPath(shootOutpost.get()),
                        Commands.waitSeconds(2),
                        AutoBuilder.followPath(outpostShoot.get()),
                        aimAndRev().withTimeout(2),
                        climbTower(shootTower));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleNeutralTowerRight() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_R-right");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_R-shoot-right");
        Optional<PathPlannerPath> shootNeutral = PathPlannerUtils.loadPathByName("outpost-neutralZone-right");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-right");

        PathPlannerAuto auto;

        var cmd = startNeutral.isEmpty() || neutralShoot.isEmpty() || shootNeutral.isEmpty() || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        aimAndRev().withTimeout(2),
                        followPathAndIntake(startNeutral, 4),
                        followPathAndStowIntake(neutralShoot),
                        followPathAndIntake(shootNeutral, 4),
                        followPathAndStowIntake(neutralShoot)
                                .andThen(aimAndRev().withTimeout(2)),
                        climbTower(shootTower));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }
}
