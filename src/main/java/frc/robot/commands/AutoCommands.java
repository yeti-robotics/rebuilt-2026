package frc.robot.commands;

import static frc.robot.constants.FieldConstants.Hub.centerHubOpening;
import static frc.robot.subsystems.feeder.FeederConfigsBeta.TEST_FEEDER_SPEED;
import static frc.robot.subsystems.indexer.IndexerConfigsBeta.TEST_INDEXER_SPEED;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConfigsBeta;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConfigsBeta;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDModes;
import frc.robot.subsystems.linslide.LinSlide;
import frc.robot.subsystems.linslide.LinSlideConfigsBeta;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.PathPlannerUtils;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AutoCommands {
    private final Climber climber;
    private final CommandSwerveDrivetrain drivetrain;
    private final Hood hood;
    private final Feeder feeder;
    private final Indexer indexer;
    private final Intake intake;
    private final LinSlide linSlide;
    private final Shooter shooter;
    private final LED led;

    public AutoCommands(
            Climber climber,
            CommandSwerveDrivetrain drivetrain,
            Hood hood,
            Indexer indexer,
            Feeder feeder,
            Intake intake,
            LinSlide linSlide,
            Shooter shooter,
            LED leds) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.indexer = indexer;
        this.feeder = feeder;
        this.intake = intake;
        this.linSlide = linSlide;
        this.shooter = shooter;
        this.led = leds;

        NamedCommands.registerCommand("rollIn", rollIn());
        NamedCommands.registerCommand("popLintake", popLintake());
    }

    // Named Commands
    public Command popLintake() {
        return linSlide.applyPower(LinSlideConfigsBeta.DEPLOY_SPEED).until(linSlide::isDeployed);
    }

    public Command rollIn() {
        return intake.applyPower(IntakeConfigsBeta.ROLLER_SPEED);
    }

    // Broken-Up Commands
    public Command intake() {
        return Commands.parallel(
                linSlide.applyPower(LinSlideConfigsBeta.DEPLOY_SPEED),
                intake.applyPower(IntakeConfigsBeta.ROLLER_SPEED));
    }

    public Command cycleNeutralRight(Optional<PathPlannerPath> pathOne, Optional<PathPlannerPath> pathTwo) {
        return Commands.sequence(
                AutoBuilder.followPath(pathOne.get())
                        .alongWith((Commands.waitSeconds(4)).andThen(intake().withTimeout(2))),
                AutoBuilder.followPath(pathTwo.get()),
                shoot());
    }

    public Command followPathAndIntake(Optional<PathPlannerPath> path, double waitTime) {
        return AutoBuilder.followPath(path.get())
                .alongWith(Commands.waitSeconds(waitTime))
                .andThen(() -> Logger.recordOutput("AutoTest", "Followed path and intaked"));
    }

    public Command followPath(Optional<PathPlannerPath> path) {
        return AutoBuilder.followPath(path.get());
    }

    public Command climbTower(Optional<PathPlannerPath> path) {
        return Commands.sequence(
                climber.deploy(ClimberConfigsBeta.CLIMBER_EXTEND_SPEED),
                AutoBuilder.followPath(path.get()),
                climber.climb(ClimberConfigsBeta.CLIMBER_RETRACT_SPEED));
    }

    public Command shoot() {
        return Commands.deadline(
                Commands.sequence(
                        new WaitCommand(0.5),
                        linSlide.applyPower(LinSlideConfigsBeta.LINSLIDE_AUTO_STOWING_SPEED)
                                .until(linSlide::isCloseToZero),
                        Commands.waitSeconds(1)),
                Commands.parallel(
                        AutoAimCommands.readyAim(drivetrain, shooter, centerHubOpening.toTranslation2d()),
                        AutoAimCommands.autoAim(drivetrain, () -> 0.0, () -> 0.0, centerHubOpening.toTranslation2d()),
                        new WaitCommand(0.4).andThen(indexer.applyPower(TEST_INDEXER_SPEED)),
                        new WaitCommand(0.4).andThen(feeder.applyPower(TEST_FEEDER_SPEED)),
                        new WaitCommand(0.4).andThen(intake.applyPower(IntakeConfigsBeta.ROLLER_SPEED))),
                led.runPattern(LEDModes.WAVE));
    }

    public Command shootBumpFire() {
        return Commands.sequence(
                shooter.revUpFlywheels(20).until(shooter::isAtSpeed),
                Commands.parallel(
                        shooter.shoot(20).withTimeout(2),
                        indexer.applyPower(TEST_INDEXER_SPEED).withTimeout(2),
                        feeder.applyPower(TEST_FEEDER_SPEED).withTimeout(2)));
    }

    // Test Commands
    public Command climberTest() {
        Optional<PathPlannerPath> climberTest = PathPlannerUtils.loadPathByName("Climb-Test");
        PathPlannerAuto auto;

        var cmd = climberTest.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        climber.deploy(ClimberConfigsBeta.CLIMBER_EXTEND_SPEED),
                        AutoBuilder.followPath(climberTest.get()),
                        Commands.waitSeconds(0.5),
                        climber.climb(ClimberConfigsBeta.CLIMBER_RETRACT_SPEED));

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    // Autos
    public Command oneCycleNeutralTowerLeft() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_L-left");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-left");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-left");

        PathPlannerAuto auto;

        var cmd = startNeutral.isEmpty() || neutralShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none()
                : Commands.sequence(followPathAndIntake(startNeutral, 0.5), followPath(neutralShoot), shoot());

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
                        followPathAndIntake(startDepot, 0.5),
                        followPath(depotShoot),
                        linSlide.runIntake(0.5, true),
                        shoot(),
                        climbTower(shootTower));

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleNeutralDepotTowerLeft() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_L-left");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-left");
        Optional<PathPlannerPath> shootDepot = PathPlannerUtils.loadPathByName("shoot-depot-left");
        Optional<PathPlannerPath> depotShoot = PathPlannerUtils.loadPathByName("depot-shoot-left");

        PathPlannerAuto auto;

        var cmd = shootDepot.isEmpty() || depotShoot.isEmpty() || startNeutral.isEmpty() || neutralShoot.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(startNeutral, 0.5),
                        followPath(neutralShoot),
                        shoot(),
                        followPathAndIntake(shootDepot, 0.2),
                        followPath(depotShoot),
                        shoot());

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
                        shoot(),
                        followPathAndIntake(startDepot, 2),
                        followPath(depotShoot),
                        shoot(),
                        followPathAndIntake(shootNeutral, 4),
                        followPath(neutralShoot),
                        shoot(),
                        climbTower(shootTower));

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleNeutralTowerLeft() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_L-left");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-left");
        Optional<PathPlannerPath> shootNeutralFarther = PathPlannerUtils.loadPathByName("shoot-neutral_L-farther-left");
        Optional<PathPlannerPath> neutralShootFarther = PathPlannerUtils.loadPathByName("neutral_L-shoot-farther-left");

        PathPlannerAuto auto;

        var cmd = startNeutral.isEmpty()
                        || neutralShoot.isEmpty()
                        || shootNeutralFarther.isEmpty()
                        || neutralShootFarther.isEmpty()
                ? Commands.none().andThen(Commands.print("Command is Empty"))
                : Commands.sequence(
                        followPathAndIntake(startNeutral, 0.5),
                        followPath(neutralShoot),
                        shoot(),
                        followPathAndIntake(shootNeutralFarther, 0.5),
                        followPath(neutralShootFarther),
                        shoot());

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
                        followPath(neutralLShoot),
                        shoot(),
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
                        shoot(),
                        followPathAndIntake(initNeutralR, 4),
                        AutoBuilder.followPath(neutralRShoot.get()),
                        shoot().withTimeout(2),
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
                                .andThen(shoot().andThen(linSlide.runIntake(-0.4, false))),
                        followPathAndIntake(shootNeutralL, 4),
                        AutoBuilder.followPath(neutralLShoot.get()),
                        shoot().withTimeout(2));
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
                                .andThen(shoot().andThen(linSlide.runIntake(-0.4, false))),
                        followPathAndIntake(shootNeutralR, 4),
                        AutoBuilder.followPath(neutralRShoot.get()),
                        shoot().withTimeout(2));
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
                        shoot(),
                        followPathAndIntake(initNeutralL, 4),
                        AutoBuilder.followPath(neutralLShoot.get()),
                        shoot().withTimeout(2),
                        followPathAndIntake(shootNeutralL, 4),
                        AutoBuilder.followPath(neutralLShoot.get()),
                        shoot().withTimeout(2));
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
                        shoot(),
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
                        shoot(),
                        followPathAndIntake(startNeutral, 4),
                        followPath(neutralShoot).andThen(shoot()),
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
                        shoot(),
                        linSlide.runIntake(0.5, true),
                        AutoBuilder.followPath(startOutpost.get()),
                        (Commands.waitSeconds(3)),
                        AutoBuilder.followPath(outpostShoot.get()),
                        shoot(),
                        linSlide.runIntake(-0.5, false),
                        climbTower(shootTower));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command cheesyLeft() {
        Optional<PathPlannerPath> cheesy1 = PathPlannerUtils.loadPathByName("cheesy_path1L");
        Optional<PathPlannerPath> cheesy2 = PathPlannerUtils.loadPathByName("cheesy_path2L");
        Optional<PathPlannerPath> cheesy3 = PathPlannerUtils.loadPathByName("cheesy_path3L");
        Optional<PathPlannerPath> cheesy4 = PathPlannerUtils.loadPathByName("cheesy_path4L");

        PathPlannerAuto auto;

        var cmd = cheesy1.isEmpty() || cheesy2.isEmpty() || cheesy3.isEmpty() || cheesy4.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(cheesy1, 0.5),
                        followPath(cheesy2),
                        shoot().withTimeout(3.5),
                        followPathAndIntake(cheesy3, 0.5),
                        followPath(cheesy4),
                        shoot());
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
                        shoot(),
                        AutoBuilder.followPath(startOutpost.get()),
                        (Commands.waitSeconds(3)),
                        AutoBuilder.followPath(outpostShoot.get()),
                        shoot(),
                        followPathAndIntake(shootNeutral, 4),
                        followPath(neutralShoot).andThen(shoot()),
                        climbTower(shootTower));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command twoCycleNeutralOutpostTowerRight() {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_R-right");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_R-shoot-right");
        Optional<PathPlannerPath> shootOutpost = PathPlannerUtils.loadPathByName("shoot-outpost-right");
        Optional<PathPlannerPath> outpostShoot = PathPlannerUtils.loadPathByName("outpost-shoot-right");

        PathPlannerAuto auto;

        var cmd = startNeutral.isEmpty() || neutralShoot.isEmpty() || shootOutpost.isEmpty() || outpostShoot.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(startNeutral, 0.5),
                        followPath(neutralShoot),
                        shoot().withTimeout(6),
                        AutoBuilder.followPath(shootOutpost.get()),
                        Commands.waitSeconds(1.5),
                        AutoBuilder.followPath(outpostShoot.get()),
                        shoot().withTimeout(6));
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
                        shoot(),
                        followPathAndIntake(startNeutral, 4),
                        followPath(neutralShoot),
                        followPathAndIntake(shootNeutral, 4),
                        followPath(neutralShoot).andThen(shoot()),
                        climbTower(shootTower));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }
}
