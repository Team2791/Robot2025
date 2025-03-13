package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class DispenserConstants {
    public static final class Motor {
        /** No reduction is applied */
        public static final double kReduction = 1;

        /** Either Brake or Coast */
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        /** Invert the follower motor */
        public static final boolean kInvertFollower = true;

        /** Moment of Inertia */
        public static final double kMoI = 0.01;
    }

    public static final class Encoder {
        public static final double kPositionFactor = MathConstants.kTau / Motor.kReduction;
        public static final double kVelocityFactor = kPositionFactor / 60;
    }

    public static final class Power {
        public static final double kDispense = 0.3;
        public static final double kSlowBack = -0.1;
        public static final double kDispenseIn = 0.15;
        public static final double kDislodge = -0.5;
        public static final double kDispenseL1 = 0.15;
    }
}
