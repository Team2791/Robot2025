package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.MathConstants.kTau;

public final class ElevatorConstants {
    public static final class Motor {
        /** Reduction factor from the motor to the belted wheel */
        public static final double kReduction = 1; // TODO: Find the actual reduction

        /** Either Brake or Coast */
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        /** Invert the follower motor */
        public static final boolean kInvertFollower = true;
    }

    public static class Encoder {
        public static final double kPositionFactor = kTau / Motor.kReduction;
        public static final double kVelocityFactor = kPositionFactor / 60;
    }

    public static class Drum {
        /** "If I turn the drum one rotation, how many meters will the elevator move?" */
        public static final double kCircumference = 0.1; // TODO: Find the actual circumference

        /** This is just math */
        public static final double kRadius = kCircumference / kTau;
    }

    /** If the height of the elevator at its lowest point is 0, what would the height be to score at each position? */
    public static class Heights {
        public static final double kIntake = 0.0;
        public static final double kL1 = Inches.of(0).in(Meters); // TODO: Find the actual heights
        public static final double kL2 = Inches.of(0).in(Meters);
        public static final double kL3 = Inches.of(0).in(Meters);
        public static final double kL4 = Inches.of(0).in(Meters);
        public static final double[] kLevels = { kIntake, kL1, kL2, kL3, kL4 };
    }

    public static final class Carriage {
        public static final double kMass = 1; // TODO: Find the actual mass
    }
}
