package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.GameConstants;

import java.util.Optional;

public class AllianceUtil {
    public static boolean invert() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public static boolean isRed() {
        return invert();
    }

    public static Pose2d recenter(Pose2d pose) {
        if (invert()) return pose.relativeTo(GameConstants.kRedOrigin);
        else return pose;
    }

    public static double factor() {
        if (invert()) return -1;
        else return 1;
    }

    public static Rotation2d facingDriver() {
        return recenter(Rotation2d.kPi);
    }

    public static Rotation2d recenter(Rotation2d rotation) {
        if (invert()) return SwerveUtil.normalizeAngle(rotation.plus(Rotation2d.kPi));
        else return rotation;
    }

    public static Pose2d[] recenter(Pose2d[] poses) {
        Pose2d[] recentered = new Pose2d[poses.length];
        for (int i = 0; i < poses.length; i++) {
            recentered[i] = recenter(poses[i]);
        }
        return recentered;
    }
}
