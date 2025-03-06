package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.SimCameraProperties;

public final class VisionConstants {
    public static final AprilTagFieldLayout kField = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final SimCameraProperties kSimCameraProps;

    static {
        kSimCameraProps = SimCameraProperties.PERFECT_90DEG(); // todo: load from json
    }

    public static final class Names {
        public static final String kFront = "front";
    }

    public static final class Transforms {
        public static final Transform3d kBotToFront = new Transform3d(); // todo: actual values
    }
}
