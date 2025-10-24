package frc.robot.constants;

/** PID constants, etc */
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

    public static final class RateLimits {
        public static final double kOrthogonal = 1.667;
        public static final double kRotation = 3.87;
    }

    public static final class Intake {
        public static final class Power {
            public static final double kIntake = 0.8;
            public static final double kOuttake = -0.8;
            public static final double kPivot = 0.5;
        }
    }
}
