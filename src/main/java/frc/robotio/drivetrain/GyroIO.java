package frc.robotio.drivetrain;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Timestamped;

public abstract class GyroIO {
	@SuppressWarnings("unchecked")
	@AutoLog
	public static class GyroData {
		public boolean connected = false;
		public Angle heading = Radians.of(0);
		public AngularVelocity velocity = RadiansPerSecond.of(0);
		public Timestamped<Angle>[] cached = new Timestamped[0];
	}

	/**
	 * The current gyro's data, since the last update() call
	 */
	public final GyroDataAutoLogged data = new GyroDataAutoLogged();

	/**
	 * Updates this.data with the current gyro data.
	 */
	public abstract void update();

	/**
	 * Resets the gyro.
	 */
	public abstract void reset();
}
