package frc.robot.constants;

import static frc.robot.constants.MathConstants.kTau;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class DriveConstants {
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
		new Translation2d(PhysicalConstants.Drivetrain.kWheelBase / 2, PhysicalConstants.Drivetrain.kTrackWidth / 2),
		new Translation2d(PhysicalConstants.Drivetrain.kWheelBase / 2, -PhysicalConstants.Drivetrain.kTrackWidth / 2),
		new Translation2d(-PhysicalConstants.Drivetrain.kWheelBase / 2, PhysicalConstants.Drivetrain.kTrackWidth / 2),
		new Translation2d(-PhysicalConstants.Drivetrain.kWheelBase / 2, -PhysicalConstants.Drivetrain.kTrackWidth / 2)
	);
}
