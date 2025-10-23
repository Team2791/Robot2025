package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeConstants {
    public static final class Motor {
        /** Constants for the intake motor */
        public static final double kIntakeReduction = 1;
        public static final IdleMode kIntakeIdleMode = IdleMode.kBrake;
        public static final double kIntakeMoI = 0.01;

        /** Constants for the pivot motor */
        public static final double kPivotReduction = 1;
        public static final IdleMode kPivotIdleMode = IdleMode.kBrake;
        public static final double kPivotMoI = 0.01;
    }

    public static final class Power {
        public static final double kIntake = 0.3;
        public static final double kDislodge = -0.1;
        public static final double kPivotUp = 0.2;
        public static final double kPivotDown = -0.2;
    }

    public static final class Range {
        /** The distance, in meters, between robot and coral station before intake runs */
        public static final double kRunIntake = 1.0;
    }
}
