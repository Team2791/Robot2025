package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Alerter;

import java.util.List;

public abstract class TagAlign extends Navigate {
    final Transform2d offset;

    int stage = 0;
    int tagId = -1;

    public TagAlign(
        Drivetrain drivetrain,
        final Transform2d offset
    ) {
        super(drivetrain);

        this.offset = offset;

        //        super.targetSupplier = () -> {
        //            System.out.println("AlignClosest: aligning to closest tag");
        //
        //            Transform2d offsetFixed = new Transform2d(
        //                offset.getX(),
        //                offset.getY(),
        //                new Rotation2d(offset.getRotation().getRadians())
        //            );
        //            Pose2d robotPose = drivetrain.getPose();
        //            Pose2d tagPose = null;
        //
        //            // get nearest tag
        //            double distance = Double.POSITIVE_INFINITY;
        //            for (int id : targetIds.get()) {
        //                Pose2d pose = VisionConstants.AprilTag.kLayout.getTagPose(id).get().toPose2d();
        //                double d = robotPose.getTranslation().getDistance(pose.getTranslation());
        //
        //                if (d < distance) {
        //                    distance = d;
        //                    tagPose = pose;
        //                    tagId = id;
        //                }
        //            }
        //
        //            if (tagPose == null) {
        //                System.out.println("AlignClosest: no provided apriltags exist");
        //                tagId = -1;
        //                return null;
        //            }
        //
        //            Transform2d toRobot = tagPose.minus(robotPose);
        //            if (!caps.x) offsetFixed = new Transform2d(toRobot.getX(), offset.getY(), offset.getRotation());
        //            if (!caps.y) offsetFixed = new Transform2d(offset.getX(), toRobot.getY(), offset.getRotation());
        //            if (!caps.rot) offsetFixed = new Transform2d(offset.getX(), offset.getY(), offset.getRotation());
        //
        //            // set target robot pose
        //            Pose2d target = tagPose.transformBy(offsetFixed);
        //
        //            double targetDist = robotPose.getTranslation().getDistance(target.getTranslation());
        //
        //            if (targetDist >= VisionConstants.Align.kMaxDistance) {
        //                System.out.println(
        //                    "AlignClosest: target too far away: "
        //                        + targetDist + "m > "
        //                        + VisionConstants.Align.kMaxDistance + "m. "
        //                        + "Exiting early"
        //                );
        //                tagId = -1;
        //                return null;
        //            } else {
        //                System.out.println("AlignClosest: aligning to tag " + tagId + " with distance " + targetDist + "m");
        //            }
        //
        //            return target;
        //        };
    }

    public abstract List<Integer> getTagIds();

    @Override
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    public Pose2d getTargetPose() {
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
            return null;
        }

        Transform2d offsetFixed = null;
        Transform2d toRobot = tagPose.minus(robot);

        if (stage == 0) offsetFixed = new Transform2d(offset.getX(), toRobot.getY(), offset.getRotation());
        else offsetFixed = offset;

        Pose2d target = tagPose.transformBy(offsetFixed);

        if (robot.getTranslation().getDistance(target.getTranslation()) >= VisionConstants.Align.kMaxDistance) {
            System.out.println("AlignClosest: target too far away:");
            tagId = -1;
            return null;
        }

        return target;
    }

    public int getTagId() {
        return tagId;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Alerter.getInstance().rumble();
    }

    @Override
    public boolean isFinished() {
        if (super.isFinished() && stage == 1) {
            return true;
        } else {
            stage = 1;
            super.initialize();
            return false;
        }
    }
}
