package frc.robotsim.drivetrain;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robotio.drivetrain.GyroIO;

public class GyroSim extends GyroIO {
	final GyroSimulation sim;

	double radians = 0;
	double omega = 0;
	final Timer timer = new Timer();

	public GyroSim(GyroSimulation sim) {
		this.sim = sim;
	}

	@Override
	public void update() {
		data.heading = sim.getGyroReading().getMeasure();
		data.velocity = sim.getMeasuredAngularVelocity();
		data.connected = false;
	}

	@Override
	public void reset() {
		sim.setRotation(new Rotation2d());
	}
}
