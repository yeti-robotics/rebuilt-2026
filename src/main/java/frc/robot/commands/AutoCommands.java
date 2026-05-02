package frc.robot.commands;

import static frc.robot.constants.FieldConstants.Hub.centerHubOpening;
import static frc.robot.subsystems.feeder.FeederConfigsBeta.FEEDER_SPEED;
import static frc.robot.subsystems.indexer.IndexerConfigsBeta.TEST_INDEXER_SPEED;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConfigsBeta;
import frc.robot.subsystems.linslide.LinSlide;
import frc.robot.subsystems.linslide.LinSlideConfigsBeta;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.PathPlannerUtils;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AutoCommands {
    private final CommandSwerveDrivetrain drivetrain;
    private final Hood hood;
    private final Feeder feeder;
    private final Indexer indexer;
    private final Intake intake;
    private final LinSlide linSlide;
    private final Shooter shooter;

    public AutoCommands(
            CommandSwerveDrivetrain drivetrain,
            Hood hood,
            Indexer indexer,
            Feeder feeder,
            Intake intake,
            LinSlide linSlide,
            Shooter shooter) {
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.indexer = indexer;
        this.feeder = feeder;
        this.intake = intake;
        this.linSlide = linSlide;
        this.shooter = shooter;

        NamedCommands.registerCommand("rollIn", rollIn());
        NamedCommands.registerCommand("popLintake", popLintake());
    }

    // Named Commands
    public Command popLintake() {
        return linSlide.applyPower(LinSlideConfigsBeta.DEPLOY_SPEED)
                .until(linSlide::isDeployed)
                .andThen(linSlide.applyPower(0.15));
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

    public Command hoodDown() {
        return hood.stowHood();
    }

    public Command shoot() {
        return Commands.parallel(
                AutoAimCommands.readyAim(drivetrain, shooter, hood, centerHubOpening.toTranslation2d()),
                AutoAimCommands.autoAim(drivetrain, () -> 0.0, () -> 0.0, centerHubOpening.toTranslation2d()),
                linSlide.applyPower(LinSlideConfigsBeta.LINSLIDE_AUTO_SHOOT_SPEED),
                shooter.switchSlot(0),
                new WaitCommand(0.4).andThen(indexer.applyPower(TEST_INDEXER_SPEED)),
                new WaitCommand(0.4).andThen(feeder.applyPower(1)),
                new WaitCommand(0.4).andThen(intake.applyPower(IntakeConfigsBeta.ROLLER_SPEED)),
                new WaitCommand(0.4).andThen(shooter.switchSlot(1)));
    }

    public Command shuttleLeft() {
        return Commands.parallel(
                AutoAimCommands.shuttleReadyAim(drivetrain, shooter, hood),
                AutoAimCommands.leftShuttleAimAuto(drivetrain),
                shooter.switchSlot(0),
                new WaitCommand(0.4).andThen(indexer.applyPower(TEST_INDEXER_SPEED)),
                new WaitCommand(0.4).andThen(feeder.feed(FEEDER_SPEED)),
                new WaitCommand(0.4).andThen(intake.applyPower(IntakeConfigsBeta.ROLLER_SPEED)),
                new WaitCommand(0.4).andThen(shooter.switchSlot(1)));
    }

    public Command shuttleRight() {
        return Commands.parallel(
                AutoAimCommands.shuttleReadyAim(drivetrain, shooter, hood),
                AutoAimCommands.rightShuttleAimAuto(drivetrain),
                shooter.switchSlot(0),
                new WaitCommand(0.4).andThen(indexer.applyPower(TEST_INDEXER_SPEED)),
                new WaitCommand(0.4).andThen(feeder.feed(FEEDER_SPEED)),
                new WaitCommand(0.4).andThen(intake.applyPower(IntakeConfigsBeta.ROLLER_SPEED)),
                new WaitCommand(0.4).andThen(shooter.switchSlot(1)));
    }

    public Command compensatePose(Pose2d pose, double velocity, double tolerance) {
        Pose2d currentPose = drivetrain.getState().Pose;
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

        double xDistance = pose.minus(currentPose).getX();
        double yDistance = pose.minus(currentPose).getY();
        Rotation2d omegaRotation = pose.minus(currentPose).getRotation();

        return Commands.sequence(
                Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.PointWheelsAt()
                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                                .withModuleDirection(pose.getRotation())))
                        .withTimeout(0.5),
                Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.RobotCentric()
                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                                .withVelocityX(velocity)))
                        .until(() -> drivetrain.getState().Pose.getMeasureX().isNear(pose.getMeasureX(), tolerance)
                                && drivetrain.getState().Pose.getMeasureY().isNear(pose.getMeasureY(), tolerance)));
    }

    // Autos
    public Command shuttleLeftAuto() {
        Optional<PathPlannerPath> startDepot = PathPlannerUtils.loadPathByName("ShuttleAuto_1L");
        Optional<PathPlannerPath> depotNeutral = PathPlannerUtils.loadPathByName("ShuttleAuto_2L");

        PathPlannerAuto auto;

        var cmd = startDepot.isEmpty() || depotNeutral.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(startDepot, 0),
                        shoot().withTimeout(2),
                        hoodDown(),
                        followPath(depotNeutral),
                        shuttleLeft());

        auto = new PathPlannerAuto(cmd);

        return auto;
    }

    public Command shuttleRightAuto() {
        Optional<PathPlannerPath> startDepot = PathPlannerUtils.loadPathByName("ShuttleAuto_1L");
        Optional<PathPlannerPath> depotNeutral = PathPlannerUtils.loadPathByName("ShuttleAuto_2R");

        PathPlannerAuto auto;

        var cmd = startDepot.isEmpty() || depotNeutral.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(startDepot, 0),
                        shoot().withTimeout(2),
                        hoodDown(),
                        followPath(depotNeutral),
                        shuttleRight());

        auto = new PathPlannerAuto(cmd);

        return auto;
    }

    public Command shuttleLeftBumpAuto() {
        Optional<PathPlannerPath> startDepot = PathPlannerUtils.loadPathByName("ShuttleAuto_1L");
        Optional<PathPlannerPath> depotBumpNeutral = PathPlannerUtils.loadPathByName("ShuttleAutoBump_2L");

        PathPlannerAuto auto;

        var cmd = startDepot.isEmpty() || depotBumpNeutral.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(startDepot, 0),
                        shoot().withTimeout(2),
                        hoodDown(),
                        followPath(depotBumpNeutral),
                        shuttleLeft());

        auto = new PathPlannerAuto(cmd);

        return auto;
    }

    public Command shuttleRightBumpAuto() {
        Optional<PathPlannerPath> startDepot = PathPlannerUtils.loadPathByName("ShuttleAuto_1L");
        Optional<PathPlannerPath> depotBumpNeutral = PathPlannerUtils.loadPathByName("ShuttleAutoBump_2R");

        PathPlannerAuto auto;

        var cmd = startDepot.isEmpty() || depotBumpNeutral.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(startDepot, 0),
                        shoot().withTimeout(2),
                        hoodDown(),
                        followPath(depotBumpNeutral),
                        shuttleRight());

        auto = new PathPlannerAuto(cmd);

        return auto;
    }

    public Command late_grabRight() {
        Optional<PathPlannerPath> late_grab1R = PathPlannerUtils.loadPathByName("late_grab1R");
        Optional<PathPlannerPath> late_grab2R = PathPlannerUtils.loadPathByName("late_grab2R");

        PathPlannerAuto auto;

        var cmd = late_grab1R.isEmpty() || late_grab2R.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        Commands.waitSeconds(3), // Default value: 2.5
                        followPath(late_grab1R),
                        Commands.waitSeconds(0), // Default value: 1.5
                        followPath(late_grab2R),
                        shoot().withTimeout(7),
                        hoodDown());

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command late_grabLeft() {
        Optional<PathPlannerPath> late_grab1L = PathPlannerUtils.loadPathByName("late_grab1L");
        Optional<PathPlannerPath> late_grab2L = PathPlannerUtils.loadPathByName("late_grab2L");

        PathPlannerAuto auto;

        var cmd = late_grab1L.isEmpty() || late_grab2L.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        Commands.waitSeconds(3), // Default value: 2.5
                        followPath(late_grab1L),
                        Commands.waitSeconds(0), // Default value: 1.5
                        followPath(late_grab2L),
                        shoot().withTimeout(7),
                        hoodDown());

        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command wraparoundRight() {
        Optional<PathPlannerPath> wraparoundR = PathPlannerUtils.loadPathByName("wraparoundR");

        PathPlannerAuto auto;

        var cmd = wraparoundR.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        Commands.waitSeconds(1.5), followPath(wraparoundR), shoot().withTimeout(2), hoodDown());

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
                        hoodDown(),
                        followPathAndIntake(cheesy3, 0.5),
                        shoot());
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command cheesyRight() {
        Optional<PathPlannerPath> cheesy1 = PathPlannerUtils.loadPathByName("cheesy_path1right");
        Optional<PathPlannerPath> cheesy2 = PathPlannerUtils.loadPathByName("cheesy_path2right");
        Optional<PathPlannerPath> cheesy3 = PathPlannerUtils.loadPathByName("cheesy_path3right");
        Optional<PathPlannerPath> cheesy4 = PathPlannerUtils.loadPathByName("cheesy_path4right");

        PathPlannerAuto auto;

        var cmd = cheesy1.isEmpty() || cheesy2.isEmpty() || cheesy3.isEmpty() || cheesy4.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(cheesy1, 0.5),
                        followPath(cheesy2),
                        shoot().withTimeout(3.5),
                        hoodDown(),
                        followPathAndIntake(cheesy3, 0.5),
                        shoot());
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command cheesyLeftBump() {
        Optional<PathPlannerPath> cheesy1 = PathPlannerUtils.loadPathByName("cheesy_path1L");
        Optional<PathPlannerPath> cheesy2 = PathPlannerUtils.loadPathByName("cheesy_path2LB");
        Optional<PathPlannerPath> cheesy3 = PathPlannerUtils.loadPathByName("cheesy_path3L");
        Optional<PathPlannerPath> cheesy4 = PathPlannerUtils.loadPathByName("cheesy_path4L");

        Pose2d cheesy2EndPose =
                cheesy2.get().getPathPoses().get(cheesy2.get().getPathPoses().size() - 1);
        Pose2d cheesy3EndPose =
                cheesy3.get().getPathPoses().get(cheesy3.get().getPathPoses().size() - 1);

        PathPlannerAuto auto;

        var cmd = cheesy1.isEmpty() || cheesy2.isEmpty() || cheesy3.isEmpty() || cheesy4.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(cheesy1, 0.5),
                        followPath(cheesy2),
                        //                        compensatePose(cheesy2EndPose, 2, 0.2),
                        shoot().withTimeout(3.5),
                        hoodDown(),
                        followPathAndIntake(cheesy3, 0.5),
                        //                        compensatePose(cheesy3EndPose, 2, 0.2),
                        shoot());
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command cheesyRightBump() {
        Optional<PathPlannerPath> cheesy1 = PathPlannerUtils.loadPathByName("cheesy_path1right");
        Optional<PathPlannerPath> cheesy2 = PathPlannerUtils.loadPathByName("cheesy_path2rightB");
        Optional<PathPlannerPath> cheesy3 = PathPlannerUtils.loadPathByName("cheesy_path3right");
        Optional<PathPlannerPath> cheesy4 = PathPlannerUtils.loadPathByName("cheesy_path4right");

        Pose2d cheesy2EndPose =
                cheesy2.get().getPathPoses().get(cheesy2.get().getPathPoses().size() - 1);
        Pose2d cheesy3EndPose =
                cheesy3.get().getPathPoses().get(cheesy3.get().getPathPoses().size() - 1);

        PathPlannerAuto auto;

        var cmd = cheesy1.isEmpty() || cheesy2.isEmpty() || cheesy3.isEmpty() || cheesy4.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        followPathAndIntake(cheesy1, 0.5),
                        followPath(cheesy2),
                        compensatePose(cheesy2EndPose, 2, 0.2),
                        shoot().withTimeout(3.5),
                        hoodDown(),
                        followPathAndIntake(cheesy3, 0.5),
                        compensatePose(cheesy3EndPose, 2, 0.2),
                        shoot());
        auto = new PathPlannerAuto(cmd);
        return auto;
    }
}
