package frc.robot.constants;

public final class PIDConstants {
	public final class DriveMotor {
		public static final double kP = 0.025;
		public static final double kI = 0.0;
		public static final double kD = 0.0002;
		public static final double kF = 0;
		public static final double kMin = -1.0;
		public static final double kMax = 1.0;
	}

	public final class TurnMotor {
		public static final double kP = 3.0;
		public static final double kI = 0.01;
		public static final double kD = 0.03;
		public static final double kF = 0.0;
		public static final double kMin = -1.0;
		public static final double kMax = 1.0;

		public static final double kTolerance = 0.01;
	}
}
