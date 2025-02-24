package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.MathConstants.kTau;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class DriveConstants {
	public static final class Dimensions {
		public static final double kWheelBase = Inches.of(21.5).in(Meters);
		public static final double kTrackWidth = Inches.of(21.5).in(Meters);
		public static final double kBumperWidth = .808; // from design
		public static final double kBumperLength = .808;
		public static final double kDriveRadius = 0.5 * Math.hypot(kWheelBase, kTrackWidth);
	}

	public static final class Slew {
		public static final double kMagnitude = 1.0;
		public static final double kRotation = 1.0;
		public static final double kDirection = 1.0;
	}

	public static final class MaxSpeed {
		public static final double kLinear = 3.6;
		public static final double kAngular = kTau;
	}

	public static final double kGyroFactor = -1.0;
	public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
		new Translation2d(Dimensions.kWheelBase / 2, Dimensions.kTrackWidth / 2),
		new Translation2d(Dimensions.kWheelBase / 2, -Dimensions.kTrackWidth / 2),
		new Translation2d(-Dimensions.kWheelBase / 2, Dimensions.kTrackWidth / 2),
		new Translation2d(-Dimensions.kWheelBase / 2, -Dimensions.kTrackWidth / 2)
	);
}
