package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.simulation.SimCameraProperties;

import java.util.List;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public final class VisionConstants {
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

    public static final class Align {
        public static final double kReefOffset = Inches.of(12.9375).div(2).in(Meters);
        public static final double kMaxDistance = 2.0;
    }

    public static final class AprilTag {
        public static final AprilTagFieldLayout kLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final List<Integer> kRedStations = List.of(1, 2);
        public static final List<Integer> kBlueStations = List.of(12, 13);

        public static final List<Integer> kRedReef = List.of(6, 7, 8, 9, 10, 11);
        public static final List<Integer> kBlueReef = List.of(17, 18, 19, 20, 21, 22);

        public static List<Integer> stations() {
            return switch (DriverStation.getAlliance().orElse(RobotConstants.kDefaultAlliance)) {
                case Blue -> kBlueStations;
                case Red -> kRedStations;
            };
        }

        public static List<Integer> reef() {
            return switch (DriverStation.getAlliance().orElse(RobotConstants.kDefaultAlliance)) {
                case Blue -> kBlueReef;
                case Red -> kRedReef;
            };
        }
    }
}
