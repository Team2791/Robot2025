package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.util.MotorState;
import org.littletonrobotics.junction.AutoLog;

public abstract class DrivetrainIO {
    public final DrivetrainDataAutoLogged data = new DrivetrainDataAutoLogged();

    /**
     * Drive the robot
     * @param speeds the chassis speeds
     * @param mode the drive mode
     */
    public abstract void drive(ChassisSpeeds speeds, DriveMode mode);

    /** @return the `Field2d` widget */
    public abstract Field2d getField2d();

    /**
     * Resets the robot's pose
     * @param pose the new pose
     */
    public abstract void resetPose(Pose2d pose);

    /**
     * Resets the robot's heading
     * @param heading the new heading
     */
    public void resetHeading(Rotation2d heading) {
        resetPose(new Pose2d(this.data.pose.getTranslation(), heading));
    }

    /** Things to do periodically */
    public abstract void update();

    public enum DriveMode {
        kFieldRelative,
        kRobotRelative,
        kTeleoperated,
    }

    @AutoLog
    public static class DrivetrainData {
        public GyroState gyro = GyroState.defaultState();

        public ModuleState frontLeft = ModuleState.defaultState();
        public ModuleState frontRight = ModuleState.defaultState();
        public ModuleState backLeft = ModuleState.defaultState();
        public ModuleState backRight = ModuleState.defaultState();

        public Pose2d pose = new Pose2d();
        public ChassisSpeeds velocity = new ChassisSpeeds();

        public record GyroState(boolean connected, double heading, double velocity) {
            public static GyroState defaultState() {
                return new GyroState(false, 0, 0);
            }
        }

        public record ModuleState(MotorState drive, MotorState angle) {
            public static ModuleState defaultState() {
                return new ModuleState(MotorState.defaultState(), MotorState.defaultState());
            }
        }
    }
}
