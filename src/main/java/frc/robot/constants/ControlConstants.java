package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.constants.MathConstants.kTau;

/** PID constants, mostly */
public final class ControlConstants {
    public static final class Drivetrain {
        public static double kMaxSpeed = 4.804;
    }

    public static final class Auto {
        public static final double kOrthoP = 1.25;
        public static final double kOrthoI = 0.00;
        public static final double kOrthoD = 0.00;

        public static final double kTurnP = 0.00;
        public static final double kTurnI = 0.00;
        public static final double kTurnD = 0.00;
    }

    public static final class Align {
        public static final double kOrthoP = 4.75;
        public static final double kOrthoI = 0.00;
        public static final double kOrthoD = 0.00;

        public static final double kTurnP = 2.60;
        public static final double kTurnI = 0.00;
        public static final double kTurnD = 0.04;

        public static final double kMaxTurnVelocity = kTau;
        public static final double kMaxTurnAcceleration = kTau;

        public static final Pose2d kTolerance = new Pose2d(0.03, 0.03, new Rotation2d(0.05));
    }

    public static final class RateLimits {
        public static final double kOrthogonal = 1.667;
        public static final double kRotation = 3.87;
    }
}
