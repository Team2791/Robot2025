package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Alerter;

import java.util.List;

public abstract class TagAlign extends SequentialCommandGroup {
    final Transform2d offset;
    final Drivetrain drivetrain;

    Pose2d targetY;
    Pose2d targetFinal;

    int tagId = -1;

    public TagAlign(Drivetrain drivetrain, Transform2d offset) {
        this.offset = offset;
        this.drivetrain = drivetrain;

        addCommands(
            new InstantCommand(this::updateTargetPose),
            new Navigate.Supplied(drivetrain, () -> targetY),
            new Navigate.Supplied(drivetrain, () -> targetFinal),
            new InstantCommand(Alerter.getInstance()::rumble)
        );
    }

    protected abstract List<Integer> getTagIds();

    @SuppressWarnings("OptionalGetWithoutIsPresent")
    private void updateTargetPose() {
        List<Integer> targetIds = getTagIds();
        Pose2d robot = drivetrain.getPose();
        Pose2d tagPose = null;

        // get nearest tag
        double distance = Double.POSITIVE_INFINITY;
        for (int id : targetIds) {
            Pose2d pose = VisionConstants.AprilTag.kLayout.getTagPose(id).get().toPose2d();
            double d = robot.getTranslation().getDistance(pose.getTranslation());

            if (d < distance) {
                distance = d;
                tagPose = pose;
                tagId = id;
            }
        }

        if (tagPose == null) {
            System.out.println("AlignClosest: no provided apriltags exist");
            tagId = -1;
            return;
        }

        Transform2d toRobot = tagPose.minus(robot);
        Transform2d offsetY = new Transform2d(toRobot.getX(), offset.getY(), offset.getRotation());

        targetY = tagPose.transformBy(offsetY);
        targetFinal = tagPose.transformBy(offset);

        if (robot.getTranslation().getDistance(targetFinal.getTranslation()) >= VisionConstants.Align.kMaxDistance) {
            System.out.println("AlignClosest: target too far away:");
            tagId = -1;
            targetY = null;
            targetFinal = null;
        }
    }

    public int getTagId() {
        return tagId;
    }
}
