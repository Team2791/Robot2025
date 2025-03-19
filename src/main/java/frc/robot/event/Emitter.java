package frc.robot.event;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.VisionConstants;

import java.util.List;
import java.util.Optional;

public class Emitter {
    public static final VoidEvent periodic = new VoidEvent();

    public static final Event<Pose2d> poseReset = new Event<>();
    public static final Event<Pose2d> poseUpdate = new Event<>(poseReset);

    public static final Event<Double> reefRange = new Event<>(
        new EventTrigger<>(
            poseUpdate,
            pose -> {
                final AprilTagFieldLayout layout = VisionConstants.AprilTag.kLayout;

                final Translation2d avg = VisionConstants.AprilTag.reef() // get current-alliance reef
                    .stream()
                    .map(layout::getTagPose) // map tag id to pose
                    .filter(Optional::isPresent)
                    .map(p -> p.get().toPose2d().getTranslation()) // get translation and average
                    .reduce(Translation2d::plus)
                    .orElse(new Translation2d())
                    .div(6);

                return pose.getTranslation().getDistance(avg);
            }
        )
    );

    public static final Event<Double> stationRange = new Event<>(
        new EventTrigger<>(
            poseUpdate,
            pose -> {
                final AprilTagFieldLayout layout = VisionConstants.AprilTag.kLayout;
                final List<Translation2d> stations = VisionConstants.AprilTag.stations()
                    .stream()
                    .map(layout::getTagPose)
                    .filter(Optional::isPresent)
                    .map(p -> p.get().toPose2d().getTranslation())
                    .toList();

                final Translation2d robot = pose.getTranslation();
                double nearest2 = Double.MAX_VALUE;

                for (Translation2d station : stations) {
                    double dx2 = Math.pow(station.getX() - robot.getX(), 2);
                    double dy2 = Math.pow(station.getY() - robot.getY(), 2);
                    double dist2 = dx2 + dy2;
                    if (dist2 < nearest2) nearest2 = dist2;
                }

                return Math.sqrt(nearest2);
            }
        )
    );
}
