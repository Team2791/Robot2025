package frc.robotsim.drivetrain;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj.Timer;
import frc.robotio.drivetrain.GyroIO;

public class GyroSim extends GyroIO {
	double radians = 0;
	double omega = 0;
	final Timer timer = new Timer();

	@Override
	public void update() {
		double dt = timer.get();
		timer.reset();
		timer.start();

		radians += omega * dt;
		data.heading = Radians.of(radians);
		data.velocity = RadiansPerSecond.of(omega);
		data.connected = false;
	}

	@Override
	public void reset() {
		timer.reset();
		radians = 0;
	}


	/**
	 * Update the desired radians per second for simulations
	 * 
	 * @param omega
	 */
	public void updateOmega(double omega) {
		this.omega = omega;
	}
}
