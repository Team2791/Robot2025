package frc.robot.subsystems.drivetrain.gyro;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IOConstants;
import frc.robot.util.Alerter;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.*;

public class NavX extends GyroIO {
    final AHRS gyro;
    Rotation2d offset = new Rotation2d();

    final Notifier thread = new Notifier(this::read);
    final Queue<Double> readings = new ArrayBlockingQueue<>(20);
    final ReentrantLock lock = new ReentrantLock();
    final double rate = 100;

    public NavX() {
        this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort, (byte) rate);
        this.gyro.enableLogging(true);

        Alerter.getInstance().registerGyro(this.gyro);
        thread.setName("GyroSensor");
        new Thread(() -> thread.startPeriodic(1.0 / rate));
    }

    void read() {
        if (!lock.tryLock()) return; // if we can't get the lock, don't read the gyro
        this.readings.add(measure().in(Radians));
        lock.unlock(); // release the lock
    }

    Angle measure() {
        return Degrees.of(gyro.getAngle() * ControlConstants.kGyroFactor).plus(offset.getMeasure());
    }

    @Override
    public void update() {
        lock.lock(); // prevent threaded writes to data during our reads.

        data.connected = gyro.isConnected();
        data.heading = this.measure();
        data.velocity = DegreesPerSecond.of(gyro.getRate() * ControlConstants.kGyroFactor);
        data.readings = readings.stream().mapToDouble(Double::doubleValue).toArray();
        readings.clear();

        lock.unlock(); // release the lock
    }

    @Override
    public void reset(Rotation2d rotation) {
        System.out.println("Resetting gyro");

        gyro.reset();
        offset = rotation;
    }
}
