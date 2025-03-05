package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public final class VisionConstants {
    public static final AprilTagFieldLayout kField = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final class Names {
        public static final String kFront = "front";
        public static final String kBack = "back";
    }

    public static final class Transforms {
        public static final Transform3d kBotToFront = new Transform3d(); // todo: actual values
    }
}
