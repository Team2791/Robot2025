package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class RobotConstants {
    public static final double kMass = 68.023;
    public static final double kMoI = 4.235;

    public static final class DriveBase {
        /** length and width between swerve modules */
        public static final double kWheelBase = Inches.of(21.5).in(Meters);
        public static final double kTrackWidth = Inches.of(21.5).in(Meters);

        /** Length and width between bumpers */
        public static final double kBumperWidth = Inches.of(31.5).in(Meters); // from design
        public static final double kBumperLength = Inches.of(31.5).in(Meters);

        /** Drive base radius */
        public static final double kDriveRadius = 0.5 * Math.hypot(kWheelBase, kTrackWidth);
    }
}
