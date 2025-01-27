package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.MathConstants.kTau;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ModuleConstants {
	public static final class Neo {
		public static final AngularVelocity kFreeSpeed = RotationsPerSecond.of(5676.0 / 60.0);
		public static final IdleMode kIdleMode = IdleMode.kBrake;
		public static final Current kCurrentLimit = Amps.of(40);
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
		public static final MomentOfInertia kMoI = KilogramSquareMeters.of(1.91e-4);
	}

	public static final class TurnMotor {
		/** Moment of intertia */
		public static final MomentOfInertia kMoI = KilogramSquareMeters.of(2.17e-5);

		/** Reduction factor */
		public static final double kReduction = 9424. / 203.;
	}

	public static final class DriveEncoder {
		public static final double kPositionFactor = Wheel.kDiameter.in(Meters) * Math.PI / DriveMotor.kReduction;
		public static final double kVelocityFactor = kPositionFactor / 60.0;
	}

	public static final class TurnEncoder {
		public static final double kPositionFactor = kTau;
		public static final double kVelocityFactor = kPositionFactor / 60.0;
		public static final boolean kInverted = true;

		public static final double kMinPidInput = 0.0;
		public static final double kMaxPidInput = kPositionFactor;

	}

	public static final class Wheel {
		public static final Distance kDiameter = Inches.of(3.0);
		public static final Distance kCircumference = kDiameter.times(Math.PI);

		public static final AngularVelocity kFreeSpeed = Neo.kFreeSpeed.div(DriveMotor.kReduction);
		public static final LinearVelocity kLinearSpeed = toLinearVel(Neo.kFreeSpeed);

		public static final Distance toLinear(Angle motorRotations) {
			return kCircumference.times(motorRotations.in(Radians)).times(DriveMotor.kReduction);
		}

		public static final LinearVelocity toLinearVel(AngularVelocity motorSpeed) {
			return kCircumference.times(motorSpeed.times(DriveMotor.kReduction).in(RadiansPerSecond))
				.div(Seconds.of(1));
		}
	}
}
