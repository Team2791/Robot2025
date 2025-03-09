package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class AlgaeManipulatorConstants {
    public static final class TurnMotor {
        public static final double kReduction = 1.0 / 25.0;

        /** Either Brake or Coast */
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        /** Moment of Inertia. TODO: find */
        public static final double kMoI = 0.01;
    }

    public static final class SpinMotor {
        /** todo: check (not super important) */
        public static final double kReduction = 0.75;

        /** Either Brake or Coast */
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        /** Moment of Inertia. TODO: find */
        public static final double kMoI = 0.01;
    }

    public static final class TurnEncoder {
        public static final double kPositionFactor = MathConstants.kTau / TurnMotor.kReduction;
        public static final double kVelocityFactor = kPositionFactor / 60;
    }

    public static final class SpinEncoder {
        public static final double kPositionFactor = MathConstants.kTau / SpinMotor.kReduction;
        public static final double kVelocityFactor = kPositionFactor / 60;
    }

    public static final class Setpoints {
        public static final double kDisabled = Degrees.of(135).in(Radians);
        public static final double kEnabled = 0;
        public static final double kPower = 0.5;
    }
}
