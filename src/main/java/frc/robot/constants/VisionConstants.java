package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.AllianceUtil;
import org.photonvision.simulation.SimCameraProperties;

import java.util.List;

import static edu.wpi.first.units.Units.*;

public final class VisionConstants {
    public static final SimCameraProperties kSimCameraProps;

    static {
        kSimCameraProps = SimCameraProperties.PERFECT_90DEG(); // todo: load from json
    }

    public static final class Names {
        public static final String kFront = "front";
        public static final String kRear = "rear";
        public static final String kDriver = "driver";
        public static final String kOrpheus = "camera";
    }

    public static final class Transforms {
        public static final Transform3d kFront = new Transform3d(
            new Translation3d(
                Inches.of(25).div(2).minus(Inches.of(5)).plus(Inches.of(7.0 / 8.0)).in(Meters),
                0,
                Inches.of(12.75).in(Meters)
            ),
            new Rotation3d()
        );
        public static final Transform3d kRear = new Transform3d(
            new Translation3d(Inches.of(-12).in(Meters), 0, Inches.of(4.75).plus(Inches.of(28.75)).in(Meters)),
            new Rotation3d(0, Degrees.of(27).in(Radians), Math.PI)
        );
        public static final Transform3d kOrpheus = new Transform3d(
            new Translation3d(Inches.of(14.75).in(Meters), 0, 0.3475),
            new Rotation3d(Math.PI, 0, 0)
        );
    }

    public static final class Align {
        public static final double kReefOffset = Inches.of(12.875).div(2).in(Meters);
        public static final double kMaxDistance = 1.50;
        public static final double kDeadline = 6.0;
    }

    public static final class AprilTag {
        public static final AprilTagFieldLayout kLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final List<Integer> kRedStations = List.of(1, 2);
        public static final List<Integer> kBlueStations = List.of(12, 13);

        public static final List<Integer> kRedReef = List.of(6, 7, 8, 9, 10, 11);
        public static final List<Integer> kBlueReef = List.of(17, 18, 19, 20, 21, 22);

        public static List<Integer> stations() {
            if (AllianceUtil.isRed()) return kRedStations;
            else return kBlueStations;
        }

        public static List<Integer> reef() {
            if (AllianceUtil.isRed()) return kRedReef;
            else return kBlueReef;
        }
    }

    public static final class StdDevs {
        public static final Matrix<N3, N1> kSingleTag = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTag = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final double kMaxDistance = 2.0;
}
