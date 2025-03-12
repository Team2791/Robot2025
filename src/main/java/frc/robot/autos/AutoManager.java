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
import frc.robot.constants.GameConstants;
import frc.robot.event.Emitter;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

import java.util.Arrays;
import java.util.List;

public class AutoManager {
    public record ScoreLocation(int offset, int level) { }

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
            drivetrain
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
        );
    }

    public Command pathfindFollow(AutoTrajectory trajectory) {
        Command toPose = trajectory.getInitialPose()
            .map(p -> ((Command) new ToNearbyPose(drivetrain, p)))
            .orElseGet(Commands::none);

        return Commands.sequence(
            toPose,
            trajectory.cmd()
        );
    }

    public Command score(Dispenser dispenser, Elevator elevator, ScoreLocation location) {
        return Commands.sequence(
            new ReefAlign(drivetrain, location.offset, List.of(19, 17, 8, 6)),
            new Elevate(elevator, location.level),
            new DispenseOut(dispenser, elevator),
            new Elevate(elevator, 0)
        );
    }

    public Command intake(Dispenser dispenser, Elevator elevator, Intake intake) {
        return Commands.sequence(
            new StationAlign(drivetrain),
            new FullIntake(dispenser, elevator, intake)
        );
    }

    public Command initialScore(
        AutoTrajectory startingToScore,
        Dispenser dispenser,
        Elevator elevator,
        ScoreLocation location
    ) {
        return Commands.sequence(
            resetFollow(startingToScore),
            score(dispenser, elevator, location)
        );
    }

    public Command intakeScoreLoop(
        AutoTrajectory scoreToIntake,
        AutoTrajectory intakeToScore,
        Dispenser dispenser,
        Elevator elevator,
        Intake intake,
        List<ScoreLocation> scoring
    ) {
        Command[] commands = new Command[4 * scoring.size()];

        int i = 0;
        for (ScoreLocation location : scoring) {
            commands[i] = pathfindFollow(scoreToIntake);
            commands[i + 1] = intake(dispenser, elevator, intake);
            commands[i + 2] = pathfindFollow(intakeToScore);
            commands[i + 3] = score(dispenser, elevator, location);

            i += 4;
        }

        return Commands.sequence(commands);
    }

    public void displayTrajectory(AutoTrajectory trajectory) {
        Pose2d[] traj = trajectory.getRawTrajectory().getPoses();

        if (GameConstants.kAllianceInvert.get()) {
            List<Pose2d> poses = Arrays.stream(traj).map(p -> p.relativeTo(GameConstants.kRedOrigin)).toList();
            drivetrain.field.getObject("AutoPath").setPoses(poses);
        } else drivetrain.field.getObject("AutoPath").setPoses(traj);
    }

    public void clearTrajectory() {
        drivetrain.field.getObject("AutoPath").setPoses();
    }

    public AutoRoutine routine(Dispenser dispenser, Elevator elevator, Intake intake) {
        AutoRoutine routine = factory.newRoutine("Main Routine");

        AutoTrajectory startingToScore = routine.trajectory("far_lscore");
        AutoTrajectory scoreToIntake = routine.trajectory("lscore_lintake");
        AutoTrajectory intakeToScore = routine.trajectory("lintake_lscore");

        startingToScore.active().onTrue(Commands.runOnce(() -> displayTrajectory(startingToScore)));
        scoreToIntake.active().onTrue(Commands.runOnce(() -> displayTrajectory(scoreToIntake)));
        intakeToScore.active().onTrue(Commands.runOnce(() -> displayTrajectory(intakeToScore)));

        startingToScore.done().onTrue(Commands.runOnce(this::clearTrajectory));
        scoreToIntake.done().onTrue(Commands.runOnce(this::clearTrajectory));
        intakeToScore.done().onTrue(Commands.runOnce(this::clearTrajectory));

        routine.active().onTrue(Commands.sequence(
            initialScore(startingToScore, dispenser, elevator, new ScoreLocation(-1, 4)),
            intakeScoreLoop(
                scoreToIntake,
                intakeToScore,
                dispenser,
                elevator,
                intake,
                List.of(
                    new ScoreLocation(1, 4),
                    new ScoreLocation(-1, 3),
                    new ScoreLocation(1, 3)
                )
            )
        ));

        return routine;
    }
}
