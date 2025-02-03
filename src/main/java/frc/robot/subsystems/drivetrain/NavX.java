package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Queue;

import edu.wpi.first.units.measure.Angle;

import com.studica.frc.AHRS;

import frc.robotio.drivetrain.GyroIO;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SignalConstants;
import frc.robot.thread.SensorThread;
import frc.robot.util.IterUtil;

public class NavX extends GyroIO {
	final AHRS gyro;
	final Queue<Angle> headings;
	final Queue<Double> timestamps;

	public NavX() {
		this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort, (int) (1. / SignalConstants.kDelay));
		this.headings = SensorThread.getInstance().register(this::heading);
		this.timestamps = SensorThread.getInstance().addTimestamps();
	}

	public Angle heading() {
		return Degrees.of(gyro.getAngle() * DriveConstants.kGyroFactor);
	}

	@Override
	public void update() {
		data.connected = gyro.isConnected();
		data.heading = this.heading();
		data.velocity = DegreesPerSecond.of(gyro.getRate() * DriveConstants.kGyroFactor);

		data.timestamps = IterUtil.toDoubleArray(timestamps.stream());
		data.headings = IterUtil.toDoubleArray(headings.stream(), Radians);

		headings.clear();
	}

	@Override
	public void reset() {
		gyro.reset();
	}
}
