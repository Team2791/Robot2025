package frc.robot.constants;

import com.studica.frc.AHRS.NavXComType;

/** Yes, this is input/output not operator interface. CANId constants, mostly. */
public final class IOConstants {
    public static final class Drivetrain {
        public static final NavXComType kGyroPort = NavXComType.kMXP_SPI;

        public static final class ModuleId {
            public static final int kFrontLeft = 1;
            public static final int kFrontRight = 2;
            public static final int kRearLeft = 3;
            public static final int kRearRight = 4;
        }

        public static final class Drive {
            public static final int kFrontLeft = 50;
            public static final int kFrontRight = 20;
            public static final int kRearLeft = 30;
            public static final int kRearRight = 40;
        }

        public static final class Turn {
            public static final int kFrontLeft = 15;
            public static final int kFrontRight = 25;
            public static final int kRearLeft = 35;
            public static final int kRearRight = 45;
        }
    }

    public static final class Controller {
        public static final int kDriver = 0;
        public static final int kOperator = 1;

        public static final double kDeadband = 0.05;
    }

    public static final class Intake {
        public static final int kIntake = 22;
        public static final int kPivot = 33;
    }
}
