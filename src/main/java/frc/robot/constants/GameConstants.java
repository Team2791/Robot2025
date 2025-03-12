package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class GameConstants {
    public static final DriverStation.Alliance kDefaultAlliance = DriverStation.Alliance.Blue;
    public static final Pose2d kInitialPose = new Pose2d(
        Inches.of(297.5).in(Meters),
        Inches.of(200.0).in(Meters),
        Rotation2d.kPi
    );
    public static final Supplier<Boolean> kAllianceInvert = () -> DriverStation.getAlliance()
        .filter(a -> a == DriverStation.Alliance.Red)
        .isPresent();

    public static final Supplier<Double> kAllianceFactor = () -> {
        if (kAllianceInvert.get()) return -1.0;
        else return 1.0;
    };

    public static final double kFieldWidth = Inches.of(317).in(Meters);
    public static final double kFieldLength = Inches.of(690.875).in(Meters);
    public static final Pose2d kRedOrigin = new Pose2d(kFieldLength, kFieldWidth, Rotation2d.kPi);
}
