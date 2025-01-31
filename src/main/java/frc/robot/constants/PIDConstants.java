package frc.robot.constants;

public final class PIDConstants {
	public final class DriveMotor {
		public static final double kP = 0.01;
		public static final double kI = 0.000;
		public static final double kD = 0.0;
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
}
