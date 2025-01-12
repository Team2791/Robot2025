package frc.robot.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.WPIUtilJNI;

// sourced from frc1108/robot2024
public class SlewWrapper {
	double currentRotation;
	double currentTranslationDir;
	double currentTranslationMag;

	SlewRateLimiter slewMagnitude;
	SlewRateLimiter slewRotation;

	double lastUpdate;

	public SlewWrapper(double magSlewRate, double rotSlewRate) {
		this.currentRotation = 0;
		this.currentTranslationDir = 0;
		this.currentTranslationMag = 0;

		this.slewMagnitude = new SlewRateLimiter(magSlewRate);
		this.slewRotation = new SlewRateLimiter(rotSlewRate);

		this.lastUpdate = WPIUtilJNI.now();
	}

	/**
	 * Update the values of the SlewWrapper
	 * 
	 * @param xspeed   the [-1, 1] value from the controller
	 * @param yspeed   the [-1, 1] value from the controller
	 * @param rotation the [-1, 1] value from the controller
	 */
	public void update(double xspeed, double yspeed, double rotation) {

	}
}
