package frc.robotio.drivetrain;

import java.security.KeyStore.Entry;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {
	@AutoLog
	public static class Inputs {
		public record MeasuredTimestamp(Angle yaw, double time) {}

		public Angle yaw;
		public AngularVelocity velocity;
		public MeasuredTimestamp[] history;
	}

	public void update(Inputs inputs);
}
