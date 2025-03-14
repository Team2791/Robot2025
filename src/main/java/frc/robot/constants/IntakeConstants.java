package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeConstants {
    public static final class Motor {
        /** No reduction is applied */
        public static final double kReduction = 1;

        /** Either Brake or Coast */
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        /** Moment of Inertia */
        public static final double kMoI = 0.01;
    }

    public static final class Power {
        public static final double kIntake = 0.3;
        public static final double kDislodge = -0.1;
    }

    public static final class Range {
        /** The distance, in meters, between robot and coral station before intake runs */
        public static final double kRunIntake = 1.0;
    }
}
