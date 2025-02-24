package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class AdvantageConstants {
	public static enum AdvantageMode {
		Real, Sim, Replay,
	}


	public static final AdvantageMode kCurrentMode = RobotBase.isReal() ? AdvantageMode.Real : AdvantageMode.Sim;
}
