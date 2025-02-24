package frc.robot.subsystems.drivetrain;

import com.studica.frc.AHRS;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.ThreadConstants;
import frc.robot.logging.threads.SensorThread;
import frc.robot.util.IterUtil;
import frc.robotio.drivetrain.GyroIO;

import java.util.Queue;

import static edu.wpi.first.units.Units.*;

public class NavX extends GyroIO {
    final AHRS gyro;
    final Queue<Angle> headings;
    final Queue<Double> timestamps;

    public NavX() {
        this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort, (int) (1. / ThreadConstants.kDelay));
        this.headings = SensorThread.getInstance().register(this::measure);
        this.timestamps = SensorThread.getInstance().makeTimestampQueue();
    }

    Angle measure() {
        return Degrees.of(gyro.getAngle() * DriveConstants.kGyroFactor);
    }

    @Override
    public void update() {
        data.connected = gyro.isConnected();
        data.heading = this.measure();
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
