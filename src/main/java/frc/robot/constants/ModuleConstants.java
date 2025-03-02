package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.MathConstants.kTau;

public final class ModuleConstants {
    public static final class AngularOffsets {
        public static final double kFrontLeft = -Math.PI / 2;
        public static final double kFrontRight = 0;
        public static final double kRearLeft = Math.PI;
        public static final double kRearRight = Math.PI / 2;
    }

    public static final class DriveMotor {
        /** Number of teeth on the pinion gear. According to docs, either 12, 13, or 14T */
        public static final double kPinionTeeth = 14.0;

        /** Number of teeth on the wheel's bevel gear */
        public static final double kBevelGearTeeth = 45.0;

        /** Number of teeth on the first-stage spur gear */
        public static final double kSpurTeeth = 22.0;

        /** Number of teeth on the bevel pinion */
        public static final double kBevelPinionTeeth = 15.0;

        /** Reduction factor from motor to wheel */
        public static final double kReduction = (kBevelGearTeeth * kSpurTeeth) / (kPinionTeeth * kBevelPinionTeeth);

        /** Moment of inertia */
        public static final double kMoI = 1.91e-4;

        /** The minimum amount of voltage that can turn the drive motor */
        public static final double kStaticFriction = 0.1;

        /** Idle mode, can be either brake or coast */
        public static final IdleMode kIdleMode = IdleMode.kBrake;

    }

    public static final class TurnMotor {
        /** Moment of inertia */
        public static final double kMoI = 2.17e-5;

        /** Reduction factor */
        public static final double kReduction = 9424. / 203.;

        /** The minimum amount of voltage that can turn the turn motor */
        public static final double kStaticFriction = 0.1;

        /** Idle mode, can be either brake or coast */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    public static final class DriveEncoder {
        /** Convert from motor-rotations to wheel-radians */
        public static final double kPositionFactor = kTau / DriveMotor.kReduction;

        /** Convert from motor rotations per minute to wheel radians per second */
        public static final double kVelocityFactor = kPositionFactor / 60.0;
    }

    public static final class TurnEncoder {
        /** Convert from motor-rotations to motor-radians */
        public static final double kPositionFactor = kTau;

        /** Convert from motor rotations per minute to motor radians per second */
        public static final double kVelocityFactor = kPositionFactor / 60.0;

        /** Invert the turn encoder. Never change this. Ever. */
        public static final boolean kInverted = true;

        public static final double kMinPidInput = 0.0;
        public static final double kMaxPidInput = kTau;
    }

    public static final class Wheel {
        /** Radius of the wheel, per design */
        public static final double kRadius = Inches.of(1.5).in(Meters);

        /** Angular free speed of the wheel */
        public static final double kFreeSpeedAngular = MotorConstants.Neo.kFreeSpeed / DriveMotor.kReduction;

        /** Linear free speed of the wheel */
        public static final double kFreeSpeedLinear = kFreeSpeedAngular * kRadius;

        /** Estimated wheel CoF, per design */
        public static final double kFrictionCoefficient = 1.3;
    }

    public static final class MaxSpeed {
        public static final double kLinear = 2.8;
        public static final double kAngular = kTau;
    }
}
