package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.MathConstants.kTau;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class ModuleConstants {
	public static final class Neo {
		/** https://www.revrobotics.com/rev-21-1650/#:~:text=empirical%20free%20speed%3A%205676%20rpm */
		public static final double kFreeSpeed = RotationsPerSecond.of(5676.0 / 60.0).in(RadiansPerSecond);
		public static final IdleMode kIdleMode = IdleMode.kBrake;
		public static final double kCurrentLimit = 40;
	}

	public static final class Neo550 {
		/** https://www.revrobotics.com/rev-21-1651/#:~:text=free%20speed%3A%2011000%20rpm */
		public static final double kFreeSpeed = RotationsPerSecond.of(11000.0 / 60.0).in(RadiansPerSecond);
		public static final IdleMode kIdleMode = IdleMode.kBrake;
	}

	public static final class DriveMotor {
		/** Number of teeth on the pinion gear. According to docs, either 12, 13, or 14T */
		public static final double kPinionTeeth = 14.0;

		/** Number of teeth on the wheel's bevel gear */
		public static final double kBevelGearTeeth = 45.0;

		/** Number of teeth on the first-stage spur gear */
		public static final double kSpurTeeth = 22.0;

		/** Number of teeth on the bevel pinion */
		public static final double kBevelPinionTeeth = 15.0;

		/** Reduction factor from motor to wheel */
		public static final double kReduction = (kBevelGearTeeth * kSpurTeeth) / (kPinionTeeth * kBevelPinionTeeth);

		/** Moment of inertia */
		public static final double kMoI = 1.91e-4;
	}

	public static final class TurnMotor {
		/** Moment of intertia */
		public static final double kMoI = 2.17e-5;

		/** Reduction factor */
		public static final double kReduction = 9424. / 203.;
	}

	public static final class DriveEncoder {
		/** Convert from motor-rotations to wheel-meters */
		public static final double kPositionFactor = Wheel.kCircumference / DriveMotor.kReduction;

		/** Convert from motor rotations per minute to wheel meters per second */
		public static final double kVelocityFactor = kPositionFactor / 60.0;
	}

	public static final class TurnEncoder {
		/** Convert from motor-rotations to motor-radians */
		public static final double kPositionFactor = kTau;

		/** Convert from motor rotations per minute to motor radians per second */
		public static final double kVelocityFactor = kPositionFactor / 60.0;

		/** May need in future, leaving */
		public static final boolean kInverted = false;

		public static final double kMinPidInput = 0.0;
		public static final double kMaxPidInput = kTau;
	}

	public static final class Wheel {
		public static final double kDiameter = Inches.of(3.0).in(Meters);
		public static final double kCircumference = kDiameter * Math.PI;
		public static final double kFreeSpeedAngular = Neo.kFreeSpeed / DriveMotor.kReduction;
		public static final double kFreeSpeedLinear = kFreeSpeedAngular * kCircumference;
	}
}
