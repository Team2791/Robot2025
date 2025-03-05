package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class DriveConstants {
    public static final double kGyroFactor = -1.0;
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
        new Translation2d(PhysicalConstants.Drivetrain.kWheelBase / 2, PhysicalConstants.Drivetrain.kTrackWidth / 2),
        new Translation2d(PhysicalConstants.Drivetrain.kWheelBase / 2, -PhysicalConstants.Drivetrain.kTrackWidth / 2),
        new Translation2d(-PhysicalConstants.Drivetrain.kWheelBase / 2, PhysicalConstants.Drivetrain.kTrackWidth / 2),
        new Translation2d(-PhysicalConstants.Drivetrain.kWheelBase / 2, -PhysicalConstants.Drivetrain.kTrackWidth / 2)
    );
}
