package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.List;
import java.util.function.Supplier;

public abstract class AlignClosest extends ToNearbyPose {
    int tagId = -1;

    @SuppressWarnings("OptionalGetWithoutIsPresent")
    public AlignClosest(Drivetrain drivetrain, Supplier<List<Integer>> targetIds, Transform2d offset) {
        super(
            drivetrain,
            new Pose2d()
        );

        super.targetSupplier = () -> {
            System.out.println("AlignClosest: aligning to closest tag");

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

            // set target robot pose
            Pose2d target = tagPose.transformBy(offset);
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

    public int getTagId() {
        return tagId;
    }
}
