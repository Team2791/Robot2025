package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class RobotConstants {
    public static final double kMass = 68.023;
    public static final double kMoI = 4.235;

    public static final class Drivetrain {
        public static final double kWheelBase = Inches.of(21.5).in(Meters);
        public static final double kTrackWidth = Inches.of(21.5).in(Meters);
        public static final double kBumperWidth = .808; // from design
        public static final double kBumperLength = .808;
        public static final double kDriveRadius = 0.5 * Math.hypot(kWheelBase, kTrackWidth);
    }

    public static final DriverStation.Alliance kDefaultAlliance = DriverStation.Alliance.Blue;
}
