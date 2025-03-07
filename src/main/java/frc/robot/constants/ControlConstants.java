package frc.robot.constants;

public final class ControlConstants {
    public static final class DriveMotor {
        public static final double kP = 0.00;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double kF = 1 / ModuleConstants.Wheel.kFreeSpeedAngular;
        public static final double kMin = -1.0;
        public static final double kMax = 1.0;
    }

    public static final class TurnMotor {
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.01;
        public static final double kF = 0.0;
        public static final double kMin = -1.0;
        public static final double kMax = 1.0;
    }

    public static final class Autos {
        public static final double kOrthoP = 2.25;
        public static final double kOrthoI = 0.00;
        public static final double kOrthoD = 0.04;

        public static final double kTurnP = 1.15;
        public static final double kTurnI = 0.00;
        public static final double kTurnD = 0.00;
    }

    public static final class Align {
        public static final double kOrthoP = 0.95;
        public static final double kOrthoI = 0.00;
        public static final double kOrthoD = 0.075;

        public static final double kTurnP = 0.50;
        public static final double kTurnI = 0.00;
        public static final double kTurnD = 0.04;
    }

    public static final class Elevator {
        public static final double kP = 0.30;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double kF = 0.0;
        public static final double kMin = -1.0;
        public static final double kMax = 1.0;
    }

    public static final class Dispenser {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        public static final double kMin = -1.0;
        public static final double kMax = 1.0;
    }

    public static final class SlewRateLimit {
        public static final double kOrthogonal = 9.5;
        public static final double kRotation = 4.5;
    }
}