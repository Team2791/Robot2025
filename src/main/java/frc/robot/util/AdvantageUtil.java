package frc.robot.util;

import java.util.function.Supplier;

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

	public static <T> T getReal(
		Supplier<T> real,
		Supplier<T> sim,
		Supplier<T> replay
	) {
		return matchReal(real, sim, replay).get();
	}
}
