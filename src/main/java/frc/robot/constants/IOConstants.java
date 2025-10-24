package frc.robot.constants;

/** Yes, this is input/output not operator interface. CAN id constants, mostly. */
public final class IOConstants {
    public static final class Arm {
        // TODO: Arm CAN IDs and constants
    }

    public static final class Intake {
        public static final int kIntake = 22;
        public static final int kPivot = 33;
    }

    public static final class Controller {
        public static final int kDriver = 0;
        public static final int kOperator = 1;

        public static final double kDeadband = 0.05;
    }
}
