package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.List;
import java.util.function.Supplier;

public abstract class TagAlign extends Navigate {
    public record Capabilities(boolean x, boolean y, boolean rot) {
        public static Capabilities all() {
            return new Capabilities(true, true, true);
        }
    }

    int tagId = -1;

    @SuppressWarnings("OptionalGetWithoutIsPresent")
    public TagAlign(
        Drivetrain drivetrain,
        Supplier<List<Integer>> targetIds,
        final Transform2d offset,
        Capabilities caps,
        NearbyPoseOptions options
    ) {
        super(
            drivetrain,
            new Pose2d(),
            options
        );

        super.targetSupplier = () -> {
            System.out.println("AlignClosest: aligning to closest tag");

            Transform2d offsetFixed = new Transform2d(
                offset.getX(),
                offset.getY(),
                new Rotation2d(offset.getRotation().getRadians())
            );
            Pose2d robotPose = drivetrain.getPose();
            Pose2d tagPose = null;

            // get nearest tag
            double distance = Double.POSITIVE_INFINITY;
            for (int id : targetIds.get()) {
                Pose2d pose = VisionConstants.AprilTag.kLayout.getTagPose(id).get().toPose2d();
                double d = robotPose.getTranslation().getDistance(pose.getTranslation());

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

            Transform2d toRobot = tagPose.minus(robotPose);
            if (!caps.x) offsetFixed = new Transform2d(toRobot.getX(), offset.getY(), offset.getRotation());
            if (!caps.y) offsetFixed = new Transform2d(offset.getX(), toRobot.getY(), offset.getRotation());
            if (!caps.rot) offsetFixed = new Transform2d(offset.getX(), offset.getY(), offset.getRotation());

            // set target robot pose
            Pose2d target = tagPose.transformBy(offsetFixed);

            double targetDist = robotPose.getTranslation().getDistance(target.getTranslation());

            if (targetDist >= VisionConstants.Align.kMaxDistance) {
                System.out.println(
                    "AlignClosest: target too far away: "
                        + targetDist + "m > "
                        + VisionConstants.Align.kMaxDistance + "m. "
                        + "Exiting early"
                );
                tagId = -1;
                return null;
            } else {
                System.out.println("AlignClosest: aligning to tag " + tagId + " with distance " + targetDist + "m");
            }

            return target;
        };
    }

    public TagAlign(
        Drivetrain drivetrain,
        Supplier<List<Integer>> targetIds,
        Transform2d offset,
        NearbyPoseOptions options
    ) {
        this(drivetrain, targetIds, offset, Capabilities.all(), options);
    }

    public TagAlign(
        Drivetrain drivetrain,
        Supplier<List<Integer>> targetIds,
        Transform2d offset
    ) {
        this(drivetrain, targetIds, offset, Capabilities.all(), new NearbyPoseOptions(true));
    }

    public int getTagId() {
        return tagId;
    }
}
