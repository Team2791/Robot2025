package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.HashMap;

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

    public static final HashMap<Integer, Integer> kTagHeights = new HashMap<>() {{
        // red side
        put(6, 2);
        put(7, 3);
        put(8, 2);
        put(9, 3);
        put(10, 2);
        put(11, 3);

        // blue side
        put(17, 2);
        put(18, 3);
        put(19, 2);
        put(20, 3);
        put(21, 2);
        put(22, 3);
    }};
}
