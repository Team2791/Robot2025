package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Alerter;
import frc.robot.util.Elastic;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;


public class ToNearbyPose extends Command {
    public record NearbyPoseOptions(boolean notifySuccess) { }

    final PIDController xController = new PIDController(
        ControlConstants.Align.kOrthoP,
        ControlConstants.Align.kOrthoI,
        ControlConstants.Align.kOrthoD
    );

    final PIDController yController = new PIDController(
        ControlConstants.Align.kOrthoP,
        ControlConstants.Align.kOrthoI,
        ControlConstants.Align.kOrthoD
    );

    final PIDController rotController = new PIDController(
        ControlConstants.Align.kTurnP,
        ControlConstants.Align.kTurnI,
        ControlConstants.Align.kTurnD
    );

    final Drivetrain drivetrain;
    final NearbyPoseOptions options;
    final Timer deadline;

    Supplier<Pose2d> targetSupplier;

    boolean exit = false;

    public ToNearbyPose(Drivetrain drivetrain, Supplier<Pose2d> targetSupplier, NearbyPoseOptions options) {
        this.drivetrain = drivetrain;
        this.options = options;
        this.targetSupplier = targetSupplier;
        this.deadline = new Timer();

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        rotController.setTolerance(0.02);

        addRequirements(drivetrain);
    }

    public ToNearbyPose(Drivetrain drivetrain, Pose2d target, NearbyPoseOptions options) {
        this(drivetrain, () -> target, options);
    }

    public ToNearbyPose(Drivetrain drivetrain, Supplier<Pose2d> target) {
        this(drivetrain, target, new NearbyPoseOptions(true));
    }

    public ToNearbyPose(Drivetrain drivetrain, Pose2d target) {
        this(drivetrain, () -> target);
    }

    @Override
    public final void initialize() {
        exit = false;
        deadline.restart();

        Pose2d target = targetSupplier.get();

        if (target == null) {
            exit = true;
            return;
        }

        xController.setSetpoint(target.getX());
        yController.setSetpoint(target.getY());
        rotController.setSetpoint(target.getRotation().getRadians());

        Logger.recordOutput("Nearby/Target", target);
        drivetrain.field.getObject("Nearby/Target").setPose(target);
    }

    @Override
    public final void execute() {
        Pose2d robot = drivetrain.getPose();

        double xPower = xController.calculate(robot.getX());
        double yPower = yController.calculate(robot.getY());
        double rotPower = rotController.calculate(robot.getRotation().getRadians());

        drivetrain.drive(xPower, yPower, rotPower, Drivetrain.FieldRelativeMode.kFixedOrigin);

        if (deadline.hasElapsed(VisionConstants.Align.kDeadline)) exit = true;
    }

    @Override
    public final void end(boolean interrupted) {
        // effectively remove target from field
        drivetrain.field.getObject("Nearby/Target").setPose(new Pose2d(-1, -1, new Rotation2d()));
        Logger.recordOutput("Nearby/Target", new Pose2d(-1, -1, new Rotation2d()));

        if (!interrupted && !exit && options.notifySuccess) {
            Alerter.getInstance().rumble();
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.Notification.NotificationLevel.INFO,
                "Alignment success",
                "Alignment has been completed successfully"
            ));
        } else if (exit) {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.Notification.NotificationLevel.WARNING,
                "Alignment cancelled",
                "Automatic alignment took too long or was too far away"
            ));
        } else {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.Notification.NotificationLevel.WARNING,
                "Alignment cancelled",
                "Automatic alignment cancelled unexpectedly"
            ));
        }
    }

    @Override
    public final boolean isFinished() {
        return exit || (xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint());
    }
}
