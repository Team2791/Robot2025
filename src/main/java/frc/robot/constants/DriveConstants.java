package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
	public static final class AngularOffsets {
		public static final Angle kFrontLeft = Radians.of(-Math.PI / 2);
		public static final Angle kFrontRight = Radians.of(0);
		public static final Angle kRearLeft = Radians.of(-Math.PI);
		public static final Angle kRearRight = Radians.of(-Math.PI / 2);
	}

	// TODO: get proper values from design ppl
	public static final class Dimensions {
		public static final Distance kWheelBase = Inches.of(20);
		public static final Distance kTrackWidth = Inches.of(20);
		public static final Distance kDriveRadius = Meters.of(
			0.5 * Math.hypot(kWheelBase.in(Meters), kTrackWidth.in(Meters))
		);
	}

	public static final class Slew {
		public static final double kMagnitude = 1.0;
		public static final double kRotation = 1.0;
	}

	public static final double kGyroFactor = 1.0;
	public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(20);
	public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
		new Translation2d(Dimensions.kWheelBase.div(2).in(Meters), Dimensions.kTrackWidth.div(2).in(Meters)),
		new Translation2d(Dimensions.kWheelBase.div(2).in(Meters), -Dimensions.kTrackWidth.div(2).in(Meters)),
		new Translation2d(-Dimensions.kWheelBase.div(2).in(Meters), Dimensions.kTrackWidth.div(2).in(Meters)),
		new Translation2d(-Dimensions.kWheelBase.div(2).in(Meters), -Dimensions.kTrackWidth.div(2).in(Meters))
	);
}
