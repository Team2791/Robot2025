package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class DriveConstants {
    public static final double kGyroFactor = -1.0;
    public static final Translation2d[] kModuleTranslations = new Translation2d[]{
        new Translation2d(RobotConstants.DriveBase.kWheelBase / 2, RobotConstants.DriveBase.kTrackWidth / 2),
        new Translation2d(RobotConstants.DriveBase.kWheelBase / 2, -RobotConstants.DriveBase.kTrackWidth / 2),
        new Translation2d(-RobotConstants.DriveBase.kWheelBase / 2, RobotConstants.DriveBase.kTrackWidth / 2),
        new Translation2d(-RobotConstants.DriveBase.kWheelBase / 2, -RobotConstants.DriveBase.kTrackWidth / 2)
    };

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(kModuleTranslations);
}
