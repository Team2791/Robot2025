package frc.robot.constants;

import com.studica.frc.AHRS.NavXComType;

/** Yes, this is input/output not operator interface. CANId constants, mostly. */
public final class IOConstants {
    public static final class Drivetrain {
        public static final class ModuleId {
            public static final int kFrontLeft = 3;
            public static final int kFrontRight = 4;
            public static final int kRearLeft = 2;
            public static final int kRearRight = 1;
        }

        public static final class Drive {
            public static final int kFrontLeft = 30;
            public static final int kFrontRight = 40;
            public static final int kRearLeft = 20;
            public static final int kRearRight = 10;
        }

        public static final class Turn {
            public static final int kFrontLeft = 35;
            public static final int kFrontRight = 45;
            public static final int kRearLeft = 25;
            public static final int kRearRight = 15;
        }

        public static final NavXComType kGyroPort = NavXComType.kUSB1;
    }

    public static final class Controller {
        public static final int kDriver = 0;
        public static final int kOperator = 1;

        public static final double kDeadband = 0.05;
    }
}
