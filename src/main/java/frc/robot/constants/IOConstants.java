package frc.robot.constants;

import com.studica.frc.AHRS.NavXComType;

/** Yes, this is input/output not operator interface. CANId constants, mostly. */
public final class IOConstants {
    public static final class Drivetrain {
        public static final class Drive {
            // Orpheus:
            //            public static final int kFrontLeft = 10;
            //            public static final int kFrontRight = 20;
            //            public static final int kRearLeft = 30;
            //            public static final int kRearRight = 40;

            // Caspian:
            public static final int kFrontLeft = 30;
            public static final int kFrontRight = 40;
            public static final int kRearLeft = 20;
            public static final int kRearRight = 10;
        }

        public static final class Turn {
            // Orpheus:
            //            public static final int kFrontLeft = 15;
            //            public static final int kFrontRight = 25;
            //            public static final int kRearLeft = 35;
            //            public static final int kRearRight = 45;

            // Caspian:
            public static final int kFrontLeft = 35;
            public static final int kFrontRight = 45;
            public static final int kRearLeft = 25;
            public static final int kRearRight = 15;
        }

        public static final NavXComType kGyroPort = NavXComType.kUSB1;
    }

    public static final class Elevator {
        public static final int kLeader = 50;
        public static final int kFollower = 55;
    }

    public static final class Dispenser {
        public static final int kLeader = 51;
        public static final int kFollower = 52;
    }

    public static final class AlgaeManipulator {
        public static final int kTurn = 53;
        public static final int kSpin = 54;
    }

    public static final class Intake {
        public static final int kLeft = 60;
        public static final int kRight = 61;
    }

    public static final class Controller {
        public static final int kDriver = 0;
        public static final int kOperator = 1;

        public static final double kDeadband = 0.05;
    }
}
