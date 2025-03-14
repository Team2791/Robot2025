package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.align.ReefAlign;
import frc.robot.commands.align.StationAlign;
import frc.robot.commands.align.ToNearbyPose;
import frc.robot.commands.dispenser.DispenseOut;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.intake.FullIntake;
import frc.robot.constants.ControlConstants;
import frc.robot.event.Emitter;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class AutoManager {
    public record ScorePlacement(int offset, int level) { }

    public record ScoreTrajectories(AutoTrajectory toIntake, AutoTrajectory toScore) { }

    public record ScoreLocation(ScorePlacement placement, ScoreTrajectories trajectories, List<Integer> reefTags) { }

    final PIDController xController = new PIDController(
        ControlConstants.Auto.kOrthoP,
        ControlConstants.Auto.kOrthoI,
        ControlConstants.Auto.kOrthoD
    );

    final PIDController yController = new PIDController(
        ControlConstants.Auto.kOrthoP,
        ControlConstants.Auto.kOrthoI,
        ControlConstants.Auto.kOrthoD
    );

    final PIDController rotController = new PIDController(
        ControlConstants.Auto.kTurnP,
        ControlConstants.Auto.kTurnI,
        ControlConstants.Auto.kTurnD
    );

    final AutoFactory factory;

    final Drivetrain drivetrain;

    public AutoManager(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.factory = new AutoFactory(
            drivetrain::getPose,
            p -> Emitter.emit(new Drivetrain.PoseResetEvent(), p),
            this::follow,
            true,
            drivetrain,
            (sample, isStart) -> {
                Logger.recordOutput("Auto/CurrentTrajectory", sample.getPoses());
                drivetrain.field.getObject("Auto/CurrentTrajectory").setPoses(sample.getPoses());
            }
        );

        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void follow(SwerveSample trajectory) {
        // get current pose
        Pose2d pose = drivetrain.getPose();

        // generate speeds
        ChassisSpeeds speeds = new ChassisSpeeds(
            trajectory.vx + xController.calculate(pose.getX(), trajectory.x),
            trajectory.vy + yController.calculate(pose.getY(), trajectory.y),
            trajectory.omega + rotController.calculate(pose.getRotation().getRadians(), trajectory.heading)
        );

        // field-relative drive
        drivetrain.drive(speeds, Drivetrain.FieldRelativeMode.kFixedOrigin);
    }

    public Command resetFollow(AutoTrajectory trajectory) {
        return Commands.sequence(
            trajectory.resetOdometry(),
            trajectory.cmd()
        ).withName("Reset and Follow");
    }

    public Command pathfindFollow(AutoTrajectory trajectory) {
        return Commands.sequence(
            new ToNearbyPose(drivetrain, () -> trajectory.getInitialPose().orElse(null)),
            trajectory.cmd()
        ).withName("Pathfind and Follow");
    }

    public Command score(Dispenser dispenser, Elevator elevator, ScorePlacement placement, List<Integer> reefTags) {
        return Commands.sequence(
            new ReefAlign(drivetrain, placement.offset, reefTags),
            new Elevate(elevator, placement.level).withTimeout(5.0),
            new DispenseOut(dispenser, elevator),
            new Elevate(elevator, 0)
        ).withName("Auto: Align+Score");
    }

    public Command intake(Dispenser dispenser, Elevator elevator, Intake intake) {
        return Commands.sequence(
            new StationAlign(drivetrain),
            new FullIntake(dispenser, elevator, intake)
        ).withName("Auto: Align+Intake");
    }

    public Command initialScoreIntake(
        Dispenser dispenser,
        Elevator elevator,
        ScoreLocation location
    ) {
        return Commands.sequence(
            resetFollow(location.trajectories.toScore),
            score(dispenser, elevator, location.placement, location.reefTags),
            pathfindFollow(location.trajectories.toIntake)
        ).withName("Auto: Initial Score+Intake");
    }

    public Command intakeScoreLoop(
        Dispenser dispenser,
        Elevator elevator,
        Intake intake,
        List<ScoreLocation> scoring
    ) {
        Command[] commands = new Command[4 * scoring.size()];

        int i = 0;
        for (ScoreLocation location : scoring) {
            commands[i] = intake(dispenser, elevator, intake);
            commands[i + 1] = pathfindFollow(location.trajectories.toScore);
            commands[i + 2] = score(dispenser, elevator, location.placement, location.reefTags);
            commands[i + 3] = pathfindFollow(location.trajectories.toIntake);

            i += 4;
        }

        return Commands.sequence(commands).withName("Auto: Intake+Score Loop");
    }

    public void clearTrajectory() {
        drivetrain.field.getObject("AutoPath").setPoses();
        Logger.recordOutput("Auto/CurrentTrajectory", new Pose2d[0]);
    }

    public AutoRoutine routine(Dispenser dispenser, Elevator elevator, Intake intake) {
        AutoRoutine routine = factory.newRoutine("Main Routine");

        AutoTrajectory startingScore = routine.trajectory("far_flscore");
        AutoTrajectory startingIntake = routine.trajectory("flscore_lintake");
        AutoTrajectory secondScore = routine.trajectory("lintake_flscore");

        startingScore.done().onTrue(Commands.runOnce(this::clearTrajectory));
        startingIntake.done().onTrue(Commands.runOnce(this::clearTrajectory));
        secondScore.done().onTrue(Commands.runOnce(this::clearTrajectory));

        routine.active().onTrue(Commands.sequence(
            initialScoreIntake(
                dispenser,
                elevator,
                new ScoreLocation(
                    new ScorePlacement(-1, 4),
                    new ScoreTrajectories(startingIntake, startingScore),
                    List.of(19, 6)
                )
            ),
            intakeScoreLoop(
                dispenser,
                elevator,
                intake,
                List.of(
                    new ScoreLocation(
                        new ScorePlacement(1, 4),
                        new ScoreTrajectories(startingIntake, secondScore),
                        List.of(19, 6)
                    )
                )
            )
        ).withName("Auto: Main Routine"));

        return routine;
    }
}
