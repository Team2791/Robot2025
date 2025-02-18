package frc.robot.constants;

import static frc.robot.constants.MathConstants.kTau;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class ElevatorConstants {
	public static final class Motor {
		public static final double kReduction = 1;
		public static final IdleMode kIdleMode = IdleMode.kBrake;
	}

	public static class Encoder {
		public static final double kPositionFactor = kTau / Motor.kReduction;
		public static final double kVelocityFactor = kPositionFactor / 60;
		public static final boolean kInverted = false;
	}
}
