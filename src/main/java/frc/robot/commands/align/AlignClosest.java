package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Alerter;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;

import static frc.robot.constants.MathConstants.kTau;

public abstract class AlignClosest extends Command {
    final Drivetrain drivetrain;
    final Supplier<List<Integer>> targetIds;
    final Transform2d offset;

    int tagId = -1;
    Pose2d tagPose = null;
    Pose2d targetPose = null;
    boolean earlyExit = false;

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

    public AlignClosest(Drivetrain drivetrain, Supplier<List<Integer>> targetIds, Transform2d offset) {
        this.drivetrain = drivetrain;
        this.offset = offset;
        this.targetIds = targetIds;

        rotController.enableContinuousInput(0, kTau);
        xController.setTolerance(0.05);
        yController.setTolerance(0.005);

        addRequirements(drivetrain);
    }

    @SuppressWarnings("OptionalGetWithoutIsPresent")
    @Override
    public void initialize() {
        Pose2d robotPose = drivetrain.getPose();

        // get nearest tag
        double distance = Double.POSITIVE_INFINITY;
        for (int id : targetIds.get()) {
            Pose2d pose = VisionConstants.AprilTag.kLayout.getTagPose(id).get().toPose2d();
            double d = robotPose.getTranslation().getDistance(pose.getTranslation());

            if (d < distance) {
                distance = d;
                tagId = id;
                tagPose = pose;
            }
        }

        // set target robot pose
        targetPose = tagPose.transformBy(offset);

        if (robotPose.getTranslation().getDistance(targetPose.getTranslation()) >= VisionConstants.Align.kMaxDistance) {
            System.out.println("AlignClosest: Target too far away");
            earlyExit = true;
            return;
        }

        // set setpoints
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        rotController.setSetpoint(targetPose.getRotation().getRadians());

        // record to akit
        Logger.recordOutput("Align/Target", targetPose);
        Logger.recordOutput("Align/Tag", tagPose);
    }

    @Override
    public void execute() {
        if (earlyExit) return;

        Pose2d currentPose = drivetrain.getPose();

        // pid
        double xPower = -xController.calculate(currentPose.getX());
        double yPower = -yController.calculate(currentPose.getY());
        double rotPower = rotController.calculate(currentPose.getRotation().getRadians());

        drivetrain.drive(xPower, yPower, rotPower);
    }

    @Override
    public void end(boolean interrupted) {
        Alerter.getInstance().rumble();
    }

    @Override
    public boolean isFinished() {
        return earlyExit || xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
    }

    public int tagId() {
        return tagId;
    }
}
