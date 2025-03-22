package frc.robot.subsystems.drivetrain.gyro;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IOConstants;
import frc.robot.util.Alerter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class NavX extends GyroIO {
    final AHRS gyro;
    Rotation2d offset = new Rotation2d();

    public NavX() {
        this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort, (byte) 50);
        this.gyro.enableLogging(true);

        Alerter.getInstance().registerGyro(this.gyro);
    }

    Angle measure() {
        return Degrees.of(gyro.getAngle() * ControlConstants.kGyroFactor).plus(offset.getMeasure());
    }

    @Override
    public void update() {
        data.connected = gyro.isConnected();
        data.heading = this.measure();
        data.velocity = DegreesPerSecond.of(gyro.getRate() * ControlConstants.kGyroFactor);
    }

    @Override
    public void reset(Rotation2d rotation) {
        System.out.println("Resetting gyro");

        gyro.reset();
        offset = rotation;
    }
}
