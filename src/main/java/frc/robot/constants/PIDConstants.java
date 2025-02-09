package frc.robot.constants;

public final class PIDConstants {
	public final class DriveMotor {
		public static final double kP = 0.3;
		public static final double kI = 0.0;
		public static final double kD = 0.015;
		public static final double kF = 0.0;
		public static final double kMin = -1.0;
		public static final double kMax = 1.0;

		// standard feedforward gains
		public static final double kS = 0.0;
		public static final double kV = 0.0;
	}

	public final class TurnMotor {
		public static final double kP = 1.0;
		public static final double kI = 0.0;
		public static final double kD = 0.01;
		public static final double kF = 0.0;
		public static final double kMin = -1.0;
		public static final double kMax = 1.0;
	}
}
