package frc.robot.subsystems.drivetrain;

import static frc.robot.constants.MathConstants.kEpsilon;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.WPIUtilJNI;

import frc.robot.util.SwerveUtil;

// sourced from frc1108/robot2024
public class SlewWrapper {
	public static class SlewOutputs {
		public final double xspeed;
		public final double yspeed;
		public final double rot;

		public SlewOutputs(double xspeed, double yspeed, double rot) {
			this.xspeed = xspeed;
			this.yspeed = yspeed;
			this.rot = rot;
		}
	}

	double prevRot;
	double prevDir;
	double prevMag;

	final SlewRateLimiter slewMag;
	final SlewRateLimiter slewRot;
	final double dirSlewRate;

	double lastUpdate;

	public SlewWrapper(double magSlewRate, double rotSlewRate, double dirSlewRate) {
		this.prevRot = 0;
		this.prevDir = 0;
		this.prevMag = 0;

		this.slewMag = new SlewRateLimiter(magSlewRate);
		this.slewRot = new SlewRateLimiter(rotSlewRate);
		this.dirSlewRate = dirSlewRate;

		this.lastUpdate = WPIUtilJNI.now() * 1e-6;
	}

	/**
	 * Update the values of the SlewWrapper
	 * 
	 * @param xspeed the [-1, 1] value from the controller for x speed
	 * @param yspeed the [-1, 1] value from the controller for y speed
	 * @param rot    the [-1, 1] value from the controller for rotation
	 */
	public SlewOutputs update(double xspeed, double yspeed, double rot) {
		double tdir = Math.atan2(yspeed, xspeed);
		double tmag = Math.hypot(xspeed, yspeed);

		double directionSlew;

		// set the direction to an arbitrarily large value if the magnitude is close to 0. we do not want to divide by 0
		if (this.prevMag <= kEpsilon) directionSlew = 500;
		else directionSlew = Math.abs(this.dirSlewRate / this.prevMag);

		double now = WPIUtilJNI.now() * 1e-6;
		double dt = now - this.lastUpdate;
		double diff = SwerveUtil.angleDifference(tdir, this.prevDir);

		if (diff < Math.PI * 0.45) {
			this.prevDir = SwerveUtil.stepTowardsAngular(this.prevDir, tdir, directionSlew * dt);
			this.prevMag = this.slewMag.calculate(tmag);
		} else if (diff > Math.PI * 0.85) {
			if (this.prevMag > kEpsilon) this.prevMag = this.slewMag.calculate(0);
			else {
				this.prevDir = SwerveUtil.normalizeAngle(this.prevDir + Math.PI);
				this.prevMag = this.slewMag.calculate(tmag);
			}
		} else {
			this.prevDir = SwerveUtil.stepTowardsAngular(this.prevDir, tdir, directionSlew * dt);
			this.prevMag = this.slewMag.calculate(0.0);
		}

		this.lastUpdate = now;

		return new SlewOutputs(
			this.prevMag * Math.cos(this.prevDir),
			this.prevMag * Math.sin(this.prevDir),
			this.slewRot.calculate(rot)
		);
	}
}
