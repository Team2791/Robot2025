package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class AdvantageConstants {
	public static enum AdvantageMode {
		Real, Sim, Replay,
	}

	public static final class Modes {
		public static final AdvantageMode kSim = AdvantageMode.Real;
		public static final AdvantageMode kReal = AdvantageMode.Real;
		public static final AdvantageMode kCurrent = RobotBase.isReal() ? kReal : kSim;
	}
}
