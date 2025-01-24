package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Hertz;

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
	private final AHRS gyro;
	private final Queue<Timestamped<Angle>> history;

	public NavX() {
		this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort, (int) SignalConstants.kRate.in(Hertz));
		this.history = SensorThread.getInstance().register(this::measure);
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
		data.cached = history.toArray(Timestamped[]::new);

		history.clear();
	}

	@Override
	public void reset() {
		gyro.reset();
	}
}
