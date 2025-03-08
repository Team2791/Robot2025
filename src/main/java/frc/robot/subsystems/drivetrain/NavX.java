package frc.robot.subsystems.drivetrain;

import com.studica.frc.AHRS;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robotio.drivetrain.GyroIO;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class NavX extends GyroIO {
    final AHRS gyro;

    public NavX() {
        this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort);
    }

    Angle measure() {
        return Degrees.of(gyro.getAngle() * DriveConstants.kGyroFactor);
    }

    @Override
    public void update() {
        data.connected = gyro.isConnected();
        data.heading = this.measure();
        data.velocity = DegreesPerSecond.of(gyro.getRate() * DriveConstants.kGyroFactor);
    }

    @Override
    public void reset() {
        System.out.println("Resetting gyro");
        gyro.reset();
    }
}
