package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.util.AllianceUtil;
import org.littletonrobotics.junction.Logger;

import java.io.File;
import java.util.List;
import java.util.Objects;

public class AutoManager {
    final PIDController xController = new PIDController(
            ControlConstants.Auto.kOrthoP, ControlConstants.Auto.kOrthoI, ControlConstants.Auto.kOrthoD);

    final PIDController yController = new PIDController(
            ControlConstants.Auto.kOrthoP, ControlConstants.Auto.kOrthoI, ControlConstants.Auto.kOrthoD);

    final PIDController rotController =
            new PIDController(ControlConstants.Auto.kTurnP, ControlConstants.Auto.kTurnI, ControlConstants.Auto.kTurnD);

    final AutoFactory factory;
    final Drivetrain drivetrain;

    public AutoManager(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.factory = new AutoFactory(
                drivetrain::getPose, drivetrain::resetPose, this::follow, true, drivetrain, (traj, starting) -> {
                    if (starting) {
                        Pose2d[] recentered = AllianceUtil.recenter(traj.getPoses());
                        Logger.recordOutput("Auto/CurrentTrajectory", recentered);
                        drivetrain
                                .getField()
                                .getObject("Auto/CurrentTrajectory")
                                .setPoses(recentered);
                    } else {
                        Logger.recordOutput("Auto/CurrentTrajectory", new Pose2d[0]);
                        drivetrain
                                .getField()
                                .getObject("Auto/CurrentTrajectory")
                                .setPoses();
                    }
                });

        String choreo = Filesystem.getDeployDirectory().getPath() + "/choreo";
        for (File f : Objects.requireNonNull(new File(choreo).listFiles())) {
            if (f.isFile() && f.getName().endsWith(".traj")) {
                factory.cache().loadTrajectory(f.getName().replaceAll(".traj", ""));
            }
        }

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
                sample.omega
                        + rotController.calculate(
                                pose.getRotation().getRadians(),
                                wants.getRotation().getRadians()));

        // field-relative drive
        drivetrain.drive(speeds, DrivetrainIO.DriveMode.kFieldRelative);
    }

    public AutoRoutine routine(List<String> trajectories) {
        AutoRoutine routine = factory.newRoutine("Main Routine");

        // TODO: autos

        return routine;
    }
}
