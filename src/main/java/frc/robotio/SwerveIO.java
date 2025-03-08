package frc.robotio;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.ModuleConstants;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public abstract class SwerveIO {
    @AutoLog
    public static class SwerveData {
        public boolean driveConnected = false;
        public Angle drivePosition = Radians.of(0); // wheel position
        public AngularVelocity driveVelocity = RadiansPerSecond.of(0); // wheel velocity
        public Voltage driveVoltage = Volts.of(0);
        public Current driveCurrent = Amps.of(0);

        public boolean turnConnected = false;
        public Angle turnPosition = Radians.of(0);
        public AngularVelocity turnVelocity = RadiansPerSecond.of(0);
        public Voltage turnVoltage = Volts.of(0);
        public Current turnCurrent = Amps.of(0);

        public SwerveModuleState desired = new SwerveModuleState();
        public AngularVelocity commanded = RadiansPerSecond.of(0);

        public LinearVelocity linearVelocity() {
            double angular = driveVelocity.in(RadiansPerSecond);
            double linear = angular * ModuleConstants.Wheel.kRadius;
            return MetersPerSecond.of(linear);
        }

        public Distance linearPosition() {
            double angular = drivePosition.in(Radians);
            double linear = angular * ModuleConstants.Wheel.kRadius;
            return Meters.of(linear);
        }
    }

    public final SwerveDataAutoLogged data = new SwerveDataAutoLogged();

    public abstract void update();

    public abstract void setDesiredState(SwerveModuleState desired);

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            data.linearPosition(),
            new Rotation2d(data.turnPosition)
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            data.linearVelocity(),
            new Rotation2d(data.turnPosition)
        );
    }
}
