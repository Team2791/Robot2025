package frc.robot.util;

import static frc.robot.constants.MathConstants.kTau;

public final class SwerveUtil {

	/**
	 * Prevent instantiation.
	 */
	private SwerveUtil() {
	}

	/**
	 * Steps towards a target value.
	 *
	 * @param current the current value
	 * @param target  the target value
	 * @param step    the step size
	 * @return the new value
	 */
	public static double stepTowards(double current, double target, double step) {
		double diff = target - current;

		if (Math.abs(diff) < step) return target;
		else if (diff > 0) return current + step;
		else return current - step;
	}

	/**
	 * Steps towards a target angle, wrapping around 0 and 2π.
	 *
	 * @param current the current angle
	 * @param target  the target angle
	 * @param step    the step size
	 * @return the new angle
	 */
	public static double stepTowardsAngular(double current, double target, double step) {
		double diff = normalizeAngle(target - current);
		double targetFixed = normalizeAngle(target);

		if (diff < step) return targetFixed;
		else if (diff > Math.PI) return normalizeAngle(current + step);
		else return normalizeAngle(current - step);
	}

	/**
	 * Unsigned minimum difference between two angles, across 0/2π boundary.
	 *
	 * @param a the first angle
	 * @param b the second angle
	 * @return the minimum difference
	 */
	public static double angleDifference(double a, double b) {
		double diff = normalizeAngle(b - a);
		return Math.min(diff, kTau - diff);
	}

	/**
	 * Normalizes an angle to be within the range [0, 2π).
	 *
	 * @param angle the angle to normalize
	 * @return the normalized angle
	 */
	public static double normalizeAngle(double angle) {
		return ((angle % kTau) + kTau) % kTau; // fix negative angles
	}
}
