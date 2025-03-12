package frc.robot.subsystems.drivetrain.gyro;

import com.studica.frc.AHRS;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IOConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class NavX extends GyroIO {
    final AHRS gyro;

    public NavX() {
        this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort);
    }

    Angle measure() {
        return Degrees.of(gyro.getAngle() * ControlConstants.kGyroFactor);
    }

    @Override
    public void update() {
        data.connected = gyro.isConnected();
        data.heading = this.measure();
        data.velocity = DegreesPerSecond.of(gyro.getRate() * ControlConstants.kGyroFactor);
    }

    @Override
    public void reset() {
        System.out.println("Resetting gyro");
        gyro.reset();
    }
}
