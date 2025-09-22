package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.constants.MathConstants.kTau;

/** PID constants, mostly */
public final class ControlConstants {
    public static final class DriveMotor {
        public static final double kP = 0.004;
        public static final double kI = 1e-10;
        public static final double kD = 0.0002;
        public static final double kF = 0.01;

        public static final double kMin = -1.0;
        public static final double kMax = 1.0;
    }

    public static final class TurnMotor {
        public static final double kP = 2.00;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double kF = 0.00;

        public static final double kMinOutput = -1.0;
        public static final double kMaxOutput = 1.0;

        public static final double kMinInput = 0;
        public static final double kMaxInput = kTau;
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

    public static final class Elevator {
        public static final double kP = 0.25;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double kF = 0.0;
        public static final double kMin = -1.0;
        public static final double kMax = 1.0;
    }

    public static final class AlgaeManipulator {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0004;
        public static final double kF = 0.0;
        public static final double kMin = -1.0;
        public static final double kMax = 1.0;
    }

    public static final class SlewRateLimit {
        public static final double kOrthogonal = 1.667;
        public static final double kRotation = 3.87;
    }

    public static final double kGyroFactor = -1.0;
}