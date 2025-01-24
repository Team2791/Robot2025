package frc.robot.util;

import frc.robot.constants.AdvantageConstants;

public final class AdvantageUtil {
	private AdvantageUtil() {}

	public static <T> T matchReal(
		T real,
		T sim,
		T replay
	) {
		switch (AdvantageConstants.Modes.kCurrent) {
			case Real:
				return real;
			case Sim:
				return sim;
			default:
				return replay;
		}
	}
}
