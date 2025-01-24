package frc.robotio.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Timestamped;

public interface GyroIO {
	@AutoLog
	public static class Inputs {
		public boolean connected;
		public Angle angle;
		public AngularVelocity velocity;
		public Timestamped<Angle>[] history;
	}

	/**
	 * Takes in Inputs by reference, and updates the values,
	 * returning nothing.
	 * 
	 * @param inputs The Inputs object to update.
	 */
	public void update(Inputs inputs);
}
