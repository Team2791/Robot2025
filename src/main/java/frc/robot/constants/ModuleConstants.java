package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.MathConstants.kTau;

public final class ModuleConstants {
    public record ModuleInfo(
        int driveId,
        int turnId,
        int moduleId,
        Translation2d translation,
        double angularOffset
    )
    {
        public int ordinal() {
            return switch (moduleId) {
                case IOConstants.Drivetrain.ModuleId.kFrontLeft -> 0;
                case IOConstants.Drivetrain.ModuleId.kFrontRight -> 1;
                case IOConstants.Drivetrain.ModuleId.kRearLeft -> 2;
                case IOConstants.Drivetrain.ModuleId.kRearRight -> 3;
                default -> throw new IllegalArgumentException("Invalid module ID");
            };
        }
    }

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
        public static final double kReduction = (kPinionTeeth * kBevelPinionTeeth) / (kBevelGearTeeth * kSpurTeeth);

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

        /** Idle mode, can be either brake or coast */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    public static final class DriveEncoder {
        /** Convert from motor-rotations to wheel-radians */
        public static final double kPositionFactor = kTau * DriveMotor.kReduction;

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
    }

    public static final class Wheel {
        /** Radius of the wheel, per design */
        public static final double kRadius = Inches.of(1.5).in(Meters);

        /** Angular free speed of the wheel */
        public static final double kFreeSpeedAngular = MotorConstants.Neo.kFreeSpeed * DriveMotor.kReduction;

        /** Linear free speed of the wheel */
        public static final double kFreeSpeedLinear = kFreeSpeedAngular * kRadius;

        /** Estimated wheel CoF, per design */
        public static final double kFrictionCoefficient = 1.3;
    }

    public static final class MaxSpeed {
        public static final double kLinear = 4.804;
        public static final double kAngular = 12.440;
    }

//    public static final class MaxAccel {
//        public static final double kLinear = 8.007;
//        public static final double kAngular = 48.074;
//    }

    /** Translation2d's to each module */
    public static final class Translations {
        public static final Translation2d kFrontLeft = new Translation2d(
            RobotConstants.DriveBase.kWheelBase / 2,
            RobotConstants.DriveBase.kTrackWidth / 2
        );
        public static final Translation2d kFrontRight = new Translation2d(
            RobotConstants.DriveBase.kWheelBase / 2,
            -RobotConstants.DriveBase.kTrackWidth / 2
        );
        public static final Translation2d kRearLeft = new Translation2d(
            -RobotConstants.DriveBase.kWheelBase / 2,
            RobotConstants.DriveBase.kTrackWidth / 2
        );
        public static final Translation2d kRearRight = new Translation2d(
            -RobotConstants.DriveBase.kWheelBase / 2,
            -RobotConstants.DriveBase.kTrackWidth / 2
        );
        public static final Translation2d[] kModules = new Translation2d[]{
            kFrontLeft,
            kFrontRight,
            kRearLeft,
            kRearRight
        };
    }

    public static final ModuleInfo kFrontLeft = new ModuleInfo(
        IOConstants.Drivetrain.Drive.kFrontLeft,
        IOConstants.Drivetrain.Turn.kFrontLeft,
        IOConstants.Drivetrain.ModuleId.kFrontLeft,
        Translations.kFrontLeft,
        AngularOffsets.kFrontLeft
    );

    public static final ModuleInfo kFrontRight = new ModuleInfo(
        IOConstants.Drivetrain.Drive.kFrontRight,
        IOConstants.Drivetrain.Turn.kFrontRight,
        IOConstants.Drivetrain.ModuleId.kFrontRight,
        Translations.kFrontRight,
        AngularOffsets.kFrontRight
    );

    public static final ModuleInfo kRearLeft = new ModuleInfo(
        IOConstants.Drivetrain.Drive.kRearLeft,
        IOConstants.Drivetrain.Turn.kRearLeft,
        IOConstants.Drivetrain.ModuleId.kRearLeft,
        Translations.kRearLeft,
        AngularOffsets.kRearLeft
    );

    public static final ModuleInfo kRearRight = new ModuleInfo(
        IOConstants.Drivetrain.Drive.kRearRight,
        IOConstants.Drivetrain.Turn.kRearRight,
        IOConstants.Drivetrain.ModuleId.kRearRight,
        Translations.kRearRight,
        AngularOffsets.kRearRight
    );

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(Translations.kModules);
}
