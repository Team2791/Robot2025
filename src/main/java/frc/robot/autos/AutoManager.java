package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.align.Navigate;
import frc.robot.commands.align.ReefAlign;
import frc.robot.commands.align.StationAlign;
import frc.robot.commands.dispenser.DispenseOut;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.intake.FullIntake;
import frc.robot.constants.ControlConstants;
import frc.robot.event.EventRegistry;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceUtil;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class AutoManager {
    public record ScorePlacement(int offset, int level) { }

    public record ScoreLocation(ScorePlacement placement, AutoTrajectory trajectory) { }

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
            EventRegistry.poseReset::emit,
            this::follow,
            true,
            drivetrain,
            (traj, starting) -> {
                if (starting) {
                    Pose2d[] recentered = AllianceUtil.recenter(traj.getPoses());
                    Logger.recordOutput("Auto/CurrentTrajectory", recentered);
                    drivetrain.getField().getObject("Auto/CurrentTrajectory").setPoses(recentered);
                } else {
                    Logger.recordOutput("Auto/CurrentTrajectory", new Pose2d[0]);
                    drivetrain.getField().getObject("Auto/CurrentTrajectory").setPoses();
                }
            }
        );

        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void follow(SwerveSample sample) {
        // get current pose
        Pose2d pose = drivetrain.getPose();
        Pose2d wants = sample.getPose();

        Logger.recordOutput("Auto/CurrentPose", pose);

        // generate speeds
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), wants.getX()),
            sample.vy + yController.calculate(pose.getY(), wants.getY()),
            sample.omega + rotController.calculate(pose.getRotation().getRadians(), wants.getRotation().getRadians())
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

    public Command navigateFollow(AutoTrajectory trajectory) {
        return Commands.sequence(
            new Navigate(drivetrain) {
                @Override
                public Pose2d getTargetPose() {
                    return trajectory.getInitialPose().orElse(null);
                }
            },
            trajectory.cmd()
        ).withName("Pathfind and Follow");
    }

    public Command score(Dispenser dispenser, Elevator elevator, ScorePlacement placement) {
        return Commands.sequence(
            new ReefAlign(drivetrain, placement.offset).withTimeout(6.0),
            new WaitCommand(0.25),
            new Elevate(elevator, placement.level),
            new WaitCommand(0.25),
            new DispenseOut(dispenser, elevator),
            new RunCommand(() -> drivetrain.drive(-0.1, 0, 0, Drivetrain.FieldRelativeMode.kOff)).withTimeout(1.0),
            new InstantCommand(() -> drivetrain.drive(0, 0, 0), drivetrain),
            new Elevate(elevator, 0)
        ).withName("Auto: Align+Score");
    }

    public Command intake(Dispenser dispenser, Elevator elevator, Intake intake) {
        return Commands.sequence(
            new StationAlign(drivetrain).withTimeout(4.0),
            new FullIntake(dispenser, elevator, intake)
        ).withName("Auto: Align+Intake");
    }

    public Command initialScore(
        Dispenser dispenser,
        Elevator elevator,
        ScoreLocation location
    ) {
        return Commands.sequence(
            resetFollow(location.trajectory),
            score(dispenser, elevator, location.placement)
        ).withName("Auto: Initial Score");
    }

    public Command initialIntake(
        Intake intake,
        Dispenser dispenser,
        Elevator elevator,
        AutoTrajectory trajectory
    ) {
        return Commands.sequence(
            navigateFollow(trajectory),
            intake(dispenser, elevator, intake)
        ).withName("Auto: Initial Intake");
    }

    public AutoRoutine routine(Dispenser dispenser, Elevator elevator, Intake intake, List<String> trajectories) {
        AutoRoutine routine = factory.newRoutine("Main Routine");

        if (trajectories.isEmpty()) {
            routine.active().onTrue(Commands.print("Auto Started: No Trajectory Selected"));
            return routine;
        }

        if (trajectories.size() == 1) {
            routine.active().onTrue(initialScore(
                dispenser,
                elevator,
                new ScoreLocation(
                    new ScorePlacement(1, 4),
                    routine.trajectory(trajectories.get(0))
                )
            ));
            return routine;
        }

        routine.active().onTrue(Commands.sequence(
            initialScore(
                dispenser,
                elevator,
                new ScoreLocation(
                    new ScorePlacement(-1, 4),
                    routine.trajectory(trajectories.get(0))
                )
            ),
            initialIntake(
                intake,
                dispenser,
                elevator,
                routine.trajectory(trajectories.get(1))
            )
        ));

        return routine;
    }
}
