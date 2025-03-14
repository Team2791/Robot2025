package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.List;
import java.util.function.Supplier;

public class AlignNearby extends ToNearbyPose {
    public record DisableDirection(boolean x, boolean y, boolean rot) {
        public static DisableDirection disableNone() {
            return new DisableDirection(false, false, false);
        }
    }

    int tagId = -1;

    @SuppressWarnings("OptionalGetWithoutIsPresent")
    public AlignNearby(
        Drivetrain drivetrain,
        Supplier<List<Integer>> targetIds,
        final Transform2d offset,
        DisableDirection disabled
    ) {
        super(
            drivetrain,
            new Pose2d()
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
            if (disabled.x) offsetFixed = new Transform2d(toRobot.getX(), offset.getY(), offset.getRotation());
            if (disabled.y) offsetFixed = new Transform2d(offset.getX(), toRobot.getY(), offset.getRotation());
            if (disabled.rot) offsetFixed = new Transform2d(offset.getX(), offset.getY(), offset.getRotation());

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

    public AlignNearby(
        Drivetrain drivetrain,
        Supplier<List<Integer>> targetIds,
        Transform2d offset
    ) {
        this(drivetrain, targetIds, offset, DisableDirection.disableNone());
    }

    public int getTagId() {
        return tagId;
    }
}
