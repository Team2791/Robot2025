package frc.robotio.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Timestamped;

public abstract class GyroIO {
	@AutoLog
	public static class GyroData {
		public boolean connected;
		public Angle heading;
		public AngularVelocity velocity;
		public Timestamped<Angle>[] cached;
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
