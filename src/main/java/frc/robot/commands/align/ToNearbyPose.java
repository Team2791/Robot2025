package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Alerter;
import frc.robot.util.Elastic;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ToNearbyPose extends Command {
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

    protected Supplier<Pose2d> targetSupplier; // allow reset in child constructor
    final Drivetrain drivetrain;
    boolean exit = false;

    public ToNearbyPose(Drivetrain drivetrain, Pose2d target) {
        this(drivetrain, () -> target);
    }

    public ToNearbyPose(Drivetrain drivetrain, Supplier<Pose2d> targetSupplier) {
        this.drivetrain = drivetrain;
        this.targetSupplier = targetSupplier;

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        rotController.setTolerance(0.02);

        addRequirements(drivetrain);
    }

    @Override
    public final void initialize() {
        exit = false;
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
    }

    @Override
    public final void end(boolean interrupted) {
        // effectively remove target from field
        drivetrain.field.getObject("Nearby/Target").setPose(new Pose2d(-1, -1, new Rotation2d()));
        Logger.recordOutput("Nearby/Target", new Pose2d(-1, -1, new Rotation2d()));

        if (!interrupted) Alerter.getInstance().rumble();
        else Elastic.sendNotification(new Elastic.Notification(
            Elastic.Notification.NotificationLevel.WARNING,
            "Alignment cancelled",
            "Automatic alignment was unexpectedly cancelled"
        ));
    }

    @Override
    public final boolean isFinished() {
        return exit || (xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint());
    }
}
