package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Queue;

import edu.wpi.first.units.measure.Angle;

import com.studica.frc.AHRS;

import frc.robot.util.Timestamped;
import frc.robotio.drivetrain.GyroIO;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SignalConstants;
import frc.robot.thread.SensorThread;

public class NavX extends GyroIO {
	final AHRS gyro;
	final Queue<Timestamped<Angle>> cache;

	public NavX() {
		this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort, (int) (1. / SignalConstants.kDelay));
		this.cache = SensorThread.getInstance().register(this::measure);
	}

	public Angle measure() {
		return Degrees.of(gyro.getAngle() * DriveConstants.kGyroFactor);
	}

	@SuppressWarnings("unchecked")
	@Override
	public void update() {
		data.connected = gyro.isConnected();
		data.heading = this.measure();
		data.velocity = DegreesPerSecond.of(gyro.getRate() * DriveConstants.kGyroFactor);
		data.cached = cache.toArray(Timestamped[]::new);

		cache.clear();
	}

	@Override
	public void reset() {
		gyro.reset();
	}
}
