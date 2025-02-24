package frc.robot.constants;

public final class PIDConstants {
	public final class DriveMotor {
		public static final double kP = 0.0425;
		public static final double kI = 0.000;
		public static final double kD = 0.001;
		public static final double kF = 1 / ModuleConstants.Wheel.kFreeSpeedLinear;
		public static final double kMin = -1.0;
		public static final double kMax = 1.0;
	}

	public final class TurnMotor {
		public static final double kP = 1.0;
		public static final double kI = 0.0;
		public static final double kD = 0.01;
		public static final double kF = 0.0;
		public static final double kMin = -1.0;
		public static final double kMax = 1.0;

		public static final double kTolerance = 0.01;
	}

	public static final class Autos {
		public static final double kOrthoP = 2.25;
		public static final double kOrthoI = 0.00;
		public static final double kOrthoD = 0.04;

		public static final double kTurnP = 1.15;
		public static final double kTurnI = 0.00;
		public static final double kTurnD = 0.00;
	}
}