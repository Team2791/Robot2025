package frc.robot.constants;

public class ArmConstants {

    public static final double kEncoderMin = -3.54;
    public static final double kEncoderMax = 28.9;

    public static final double kMinAngle = -11.0;
    public static final double kMaxAngle = 90.0;

    public static final double kPosition = Math.toRadians(kMinAngle);

    public static final double kMaxSpeed = 0.15;
    public static final double kMaxAccel = 0.1;

    public static final double kEncMin = 0.0;
    public static final double kEncMax = 42.4;

    public static final double kMin = 0.0;
    public static final double kMax = 100.0;

    public static final double kPositionFactor = (kMax - kMin) / (kEncMax - kEncMin);
    public static final double kSpeed = 0.6;

    public static final double kValueTolerance = 5.0;

    public static final double kAmpAngle = 70;
    public static final double kIntakeAngle = 20;

    public static final double kPivotHeight = 0.0;
    public static final double kRobotToPivot = 0.0;
    public static final double kShintakeAngle = 0.0;
    public static final double kLength = 0.0;
}
