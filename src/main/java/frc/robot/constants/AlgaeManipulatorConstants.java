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
        public static final double kPositionFactor = MathConstants.kTau * TurnMotor.kReduction;
        public static final double kVelocityFactor = kPositionFactor / 60;
    }

    public static final class SpinEncoder {
        public static final double kPositionFactor = MathConstants.kTau * SpinMotor.kReduction;
        public static final double kVelocityFactor = kPositionFactor / 60;
    }

    public static final class Setpoints {
        public static final double kDisabled = Degrees.of(80.7).in(Radians);
        public static final double kEnabled = Degrees.of(30.0).in(Radians);
        public static final double kPower = 0.5;
    }

    public static final HashMap<Integer, Integer> kTagHeights = new HashMap<>() {{
        // red side
        put(6, 0);
        put(7, 2);
        put(8, 0);
        put(9, 2);
        put(10, 0);
        put(11, 2);

        // blue side
        put(17, 0);
        put(18, 2);
        put(19, 0);
        put(20, 2);
        put(21, 0);
        put(22, 2);
    }};
}
