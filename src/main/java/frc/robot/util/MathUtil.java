package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class MathUtil {
    public static Transform2d transformationOf(Pose2d a, Pose2d b) {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        double omega = b.getRotation().minus(a.getRotation()).getRadians();

        double cosA = Math.cos(a.getRotation().getRadians());
        double sinA = Math.sin(a.getRotation().getRadians());

        double dxA = cosA * dx + sinA * dy;
        double dyA = -sinA * dx + cosA * dy;
        return new Transform2d(dxA, dyA, new Rotation2d(omega));
    }
}
