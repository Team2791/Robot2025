package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.MathConstants.kTau;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public final class ModuleConstants {
	public static final class Spark {
		public static final AngularVelocity kFreeSpeed = RotationsPerSecond.of(5676.0 / 60.0);
		public static final IdleMode kIdleMode = IdleMode.kBrake;
		public static final Current kCurrentLimit = Amps.of(40);
	}

	public static final class DriveMotor {
		public static final double kPinionTeeth = 14.0;
		public static final double kBevelGearTeeth = 45.0;
		public static final double kSpurTeeth = 22.0;
		public static final double kBevelPinionTeeth = 15.0;
		public static final double kReduction = (kBevelGearTeeth * kSpurTeeth) / (kPinionTeeth * kBevelPinionTeeth);
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
		public static final AngularVelocity kFreeSpeed = Spark.kFreeSpeed
			.times(kCircumference.in(Meters))
			.div(DriveMotor.kReduction);
	}

}
