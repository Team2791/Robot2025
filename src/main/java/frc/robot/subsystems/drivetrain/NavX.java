package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Queue;

import edu.wpi.first.units.measure.Angle;

import com.studica.frc.AHRS;

import frc.robotio.drivetrain.GyroIO;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SignalConstants;
import frc.robot.thread.SensorThread;

public class NavX extends GyroIO {
	final AHRS gyro;
	final Queue<Angle> measurements;
	final Queue<Double> timestamps;

	public NavX() {
		this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort, (int) (1. / SignalConstants.kDelay));
		this.measurements = SensorThread.getInstance().register(this::measure);
		this.timestamps = SensorThread.getInstance().addTimestamps();
	}

	public Angle measure() {
		return Degrees.of(gyro.getAngle() * DriveConstants.kGyroFactor);
	}

	@Override
	public void update() {
		data.connected = gyro.isConnected();
		data.heading = this.measure();
		data.velocity = DegreesPerSecond.of(gyro.getRate() * DriveConstants.kGyroFactor);

		measurements.clear();
	}

	@Override
	public void reset() {
		gyro.reset();
	}
}
