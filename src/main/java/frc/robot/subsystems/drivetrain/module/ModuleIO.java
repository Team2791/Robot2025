package frc.robot.subsystems.drivetrain.module;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.ModuleConstants;
import frc.robot.util.SwerveUtil;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public abstract class ModuleIO {
    @AutoLog
    public static class ModuleData {
        public boolean driveConnected = false;
        public Distance drivePosition = Meters.of(0);
        public LinearVelocity driveVelocity = MetersPerSecond.of(0);
        public Voltage driveVoltage = Volts.of(0);
        public Current driveCurrent = Amps.of(0);

        public boolean turnConnected = false;
        public Angle turnPosition = Radians.of(0);
        public AngularVelocity turnVelocity = RadiansPerSecond.of(0);
        public Voltage turnVoltage = Volts.of(0);
        public Current turnCurrent = Amps.of(0);
    }

    public final ModuleDataAutoLogged data = new ModuleDataAutoLogged();
    public final ModuleConstants.ModuleInfo info;

    protected ModuleIO(ModuleConstants.ModuleInfo info) {
        this.info = info;
    }

    public abstract void update();

    /** Set the desired state of the module. This includes velocity and position */
    public void setDesiredState(SwerveModuleState desired) {
        desired.optimize(new Rotation2d(data.turnPosition));
        desired.cosineScale(new Rotation2d(data.turnPosition));

        double commanded = desired.speedMetersPerSecond;
        double turn = SwerveUtil.normalizeAngle(desired.angle.getRadians());

        setStateSetpoint(commanded, turn);

        Logger.recordOutput("Drivetrain/Module/%d/DesiredSpeed".formatted(info.moduleId()), commanded);
        Logger.recordOutput("Drivetrain/Module/%d/DesiredAngle".formatted(info.moduleId()), turn);
    }

    /** Set the desired state of the module. This includes velocity and position */
    public abstract void setStateSetpoint(double driveVelocity, double turnPosition);

    /** Change from either coast or brake. */
    public abstract void setIdleMode(SparkBaseConfig.IdleMode mode);

    /** Run the drive motor at the specified open loop value. */
    public abstract void driveOpenLoop(double output);

    /** Run the turn motor at the specified open loop value. */
    public abstract void zeroTurn();

    public final SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            data.drivePosition,
            new Rotation2d(data.turnPosition)
        );
    }

    public final SwerveModuleState getState() {
        return new SwerveModuleState(
            data.driveVelocity,
            new Rotation2d(data.turnPosition)
        );
    }
}
