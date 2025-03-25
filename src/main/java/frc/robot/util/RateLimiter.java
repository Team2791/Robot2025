package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class RateLimiter {
    public record Outputs(double xspeed, double yspeed, double rot) { }

    final SlewRateLimiter xLimiter;
    final SlewRateLimiter yLimiter;
    final SlewRateLimiter rotLimiter;

    public RateLimiter(double xRate, double yRate, double rotRate) {
        xLimiter = new SlewRateLimiter(xRate);
        yLimiter = new SlewRateLimiter(yRate);
        rotLimiter = new SlewRateLimiter(rotRate);
    }

    /**
     * Calculate the new speeds based on the controller inputs
     *
     * @param xspeed the [-1, 1] value from the controller for x speed
     * @param yspeed the [-1, 1] value from the controller for y speed
     * @param rot    the [-1, 1] value from the controller for rotation
     */
    public Outputs calculate(double xspeed, double yspeed, double rot) {
        return new Outputs(
            xLimiter.calculate(xspeed),
            yLimiter.calculate(yspeed),
            rotLimiter.calculate(rot)
        );
    }
}
