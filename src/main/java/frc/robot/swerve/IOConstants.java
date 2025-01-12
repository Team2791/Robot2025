package frc.robot.swerve;

import com.studica.frc.AHRS.NavXComType;

/**
 * Yes, this is input/output not operator interface.
 */
public final class IOConstants {
	public static final class Drivetrain {
		public static final class Drive {
			public static final int kFrontLeft = 40;
			public static final int kFrontRight = 30;
			public static final int kRearLeft = 20;
			public static final int kRearRight = 10;
		}

		public static final class Turn {
			public static final int kFrontLeft = 45;
			public static final int kFrontRight = 35;
			public static final int kRearLeft = 25;
			public static final int kRearRight = 15;
		}

		public static final NavXComType kGyro = NavXComType.kMXP_SPI;
	}

	public static final class Controller {
		public static final double kDeadband = 0.1;
	}
}
