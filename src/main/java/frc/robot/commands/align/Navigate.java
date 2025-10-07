package frc.robot.commands.align;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.constants.MathConstants.kTau;

public abstract class Navigate extends Command {
    final HolonomicDriveController controller;
    final Drivetrain drivetrain;
    PIDController xController = new PIDController(
            ControlConstants.Align.kOrthoP, ControlConstants.Align.kOrthoI, ControlConstants.Align.kOrthoD);
    PIDController yController = new PIDController(
            ControlConstants.Align.kOrthoP, ControlConstants.Align.kOrthoI, ControlConstants.Align.kOrthoD);
    ProfiledPIDController rotController = new ProfiledPIDController(
            ControlConstants.Align.kTurnP,
            ControlConstants.Align.kTurnI,
            ControlConstants.Align.kTurnD,
            new TrapezoidProfile.Constraints(
                    ControlConstants.Align.kMaxTurnVelocity, ControlConstants.Align.kMaxTurnAcceleration));
    Pose2d currentTarget;
    boolean exit = false;

    public Navigate(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.controller = new HolonomicDriveController(xController, yController, rotController);

        rotController.enableContinuousInput(0, kTau);
        controller.setTolerance(ControlConstants.Align.kTolerance);

        addRequirements(drivetrain);
    }

    protected abstract Pose2d getTargetPose();

    @Override
    public final void initialize() {
        exit = false;
        currentTarget = getTargetPose();

        if (currentTarget == null) {
            exit = true;
            return;
        }

        Logger.recordOutput("Nearby/Target", currentTarget);
        drivetrain.getField().getObject("Nearby/Target").setPose(currentTarget);
    }

    @Override
    public final void execute() {
        if (currentTarget == null) return;

        Pose2d robot = drivetrain.getData().pose;
        ChassisSpeeds speeds = controller.calculate(robot, currentTarget, 0.01, currentTarget.getRotation());
        drivetrain.drive(speeds, DrivetrainIO.DriveMode.kRobotRelative);
    }

    @Override
    @OverridingMethodsMustInvokeSuper
    public void end(boolean interrupted) {
        // effectively remove target from field
        drivetrain.getField().getObject("Nearby/Target").setPose(new Pose2d(-1, -1, new Rotation2d()));
        Logger.recordOutput("Nearby/Target", new Pose2d(-1, -1, new Rotation2d()));

        drivetrain.drive(new ChassisSpeeds(), DrivetrainIO.DriveMode.kRobotRelative);
    }

    @Override
    public boolean isFinished() {
        return controller.atReference();
    }

    public static class Supplied extends Navigate {
        final Supplier<Pose2d> target;

        public Supplied(Drivetrain drivetrain, Supplier<Pose2d> target) {
            super(drivetrain);
            this.target = target;
        }

        @Override
        protected Pose2d getTargetPose() {
            return target.get();
        }
    }
}
