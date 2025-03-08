package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.*;
import frc.robot.event.Emitter;
import frc.robot.util.IterUtil;
import frc.robotio.drivetrain.GyroIO;
import frc.robotio.drivetrain.SwerveIO;
import frc.robotio.photon.CameraIO;
import org.dyn4j.geometry.Vector2;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.Arrays;
import java.util.function.Function;
import java.util.stream.Stream;

public class Drivetrain extends SubsystemBase {
    public static class PoseUpdateEvent extends Emitter.Event<Pose2d> {
        @Override
        public Emitter.Dependency<Pose2d, Pose2d> runAfter() {
            return new Emitter.Dependency<>(new PoseResetEvent(), Function.identity());
        }
    }

    public static class PoseResetEvent extends Emitter.Event<Pose2d> { }

    final SwerveIO frontLeft;
    final SwerveIO frontRight;
    final SwerveIO rearLeft;
    final SwerveIO rearRight;

    final GyroIO gyro;

    final SwerveDrivePoseEstimator odometry;
    final Field2d field;

    final RateLimiter slew;

    public Drivetrain(
        GyroIO gyro,
        Function<Integer, SwerveIO> moduleConstructor
    ) throws IOException, ParseException {
        this.gyro = gyro;

        frontLeft = moduleConstructor.apply(0);
        frontRight = moduleConstructor.apply(1);
        rearLeft = moduleConstructor.apply(2);
        rearRight = moduleConstructor.apply(3);

        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kKinematics,
            gyro.heading(),
            modulePositions(),
            RobotConstants.kInitialPose
        );

        slew = new RateLimiter(
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kRotation
        );

        field = new Field2d();

        this.gyro.reset();

        AutoBuilder.configure(
            this::getPose,
            (pose) -> Emitter.emit(new PoseResetEvent(), pose),
            this::getChassisSpeeds,
            s -> this.drive(s, false),
            new PPHolonomicDriveController(
                new com.pathplanner.lib.config.PIDConstants(
                    ControlConstants.Autos.kOrthoP,
                    ControlConstants.Autos.kOrthoI,
                    ControlConstants.Autos.kOrthoD
                ),
                new com.pathplanner.lib.config.PIDConstants(
                    ControlConstants.Autos.kTurnP,
                    ControlConstants.Autos.kTurnI,
                    ControlConstants.Autos.kTurnD
                )
            ),
            RobotConfig.fromGUISettings(),
            () -> DriverStation.getAlliance().map(al -> al == DriverStation.Alliance.Red).orElse(false),
            this
        );

        PathPlannerLogging.setLogActivePathCallback((path) -> {
            Logger.recordOutput("Autos/Path", path.toArray(Pose2d[]::new));
            field.getObject("Autos/Path").setPoses(path);
        });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            Logger.recordOutput("Autos/TargetPose", pose);
            field.getObject("Autos/TargetPose").setPose(pose);
        });

        // Elastic SwerveDrive widget
        SmartDashboard.putData(
            "SwerveDrive",
            builder -> {
                builder.setSmartDashboardType("SwerveDrive");

                IterUtil.zipThen(
                    Arrays.stream(modules()),
                    Stream.of("Front Left", "Front Right", "Back Left", "Back Right"),
                    (module, label) -> {
                        builder.addDoubleProperty(
                            label + " Angle",
                            () -> module.getState().angle.getRadians(),
                            null
                        );
                        builder.addDoubleProperty(
                            label + " Velocity",
                            () -> module.getState().speedMetersPerSecond,
                            null
                        );
                    }
                );

                builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
            }
        );

        AutoLogOutputManager.addObject(this);
        Emitter.on(new PoseUpdateEvent(), field::setRobotPose);
        Emitter.on(new PoseResetEvent(), pose -> odometry.resetPosition(gyro.heading(), modulePositions(), pose));
    }

    /**
     * @return A list of all swerve modules on the robot. frontLeft, frontRight, rearLeft, rearRight in that order.
     */
    public SwerveIO[] modules() {
        return new SwerveIO[]{
            frontLeft, frontRight, rearLeft, rearRight
        };
    }

    /**
     * @return A list of all swerve module positions on the robot. In the same order as {@link #modules()}.
     */
    @AutoLogOutput
    public SwerveModulePosition[] modulePositions() {
        return Arrays.stream(modules()).map(SwerveIO::getPosition).toArray(SwerveModulePosition[]::new);
    }

    /**
     * @return A list of all swerve module states on the robot. In the same order as {@link #modules()}.
     */
    public SwerveModuleState[] moduleStates() {
        return Arrays.stream(modules()).map(SwerveIO::getState).toArray(SwerveModuleState[]::new);
    }

    /**
     * @return The speeds of the entire chassis
     */
    public ChassisSpeeds getChassisSpeeds() { return DriveConstants.kKinematics.toChassisSpeeds(moduleStates()); }

    /**
     * @param speeds The desired speeds for the robot to move at.
     */
    private void setDesiredSpeeds(ChassisSpeeds speeds) {
        // according to delphi, this should remove some skew
        ChassisSpeeds discrete = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states = DriveConstants.kKinematics.toSwerveModuleStates(discrete);

        // this probably doesn't need to happen again but just in case we get bad parameters somehow
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.MaxSpeed.kLinear);

        IterUtil.zipThen(Arrays.stream(modules()), Arrays.stream(states), SwerveIO::setDesiredState);
    }

    /**
     * @return The estimated pose of the robot.
     */
    @AutoLogOutput
    public Pose2d getPose() { return odometry.getEstimatedPosition(); }

    public Rotation2d getHeading() { return getPose().getRotation(); }

    /**
     * Swerve drive control
     *
     * @param speeds        The desired speeds for the robot to move at.
     * @param fieldRelative Whether the speeds are field-relative or robot-relative. Defaults to true.
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        if (fieldRelative) setDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading()));
        else setDesiredSpeeds(speeds);
    }

    /**
     * Field-relative swerve drive control
     *
     * @param speeds The desired speeds for the robot to move at.
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, true);
    }

    /**
     * Manual swerve drive control
     *
     * @param xPower        The desired x-direction power. +X is forward, must be [-1, 1]
     * @param yPower        The desired y-direction power. +Y is left, must be [-1, 1]
     * @param rotPower      The desired rotational power. +R is ccw, must be [-1, 1]
     * @param fieldRelative Whether the speeds are field-relative or robot-relative. Defaults to true.
     */
    public void drive(double xPower, double yPower, double rotPower, boolean fieldRelative) {
        Vector2 velocity = new Vector2(xPower, yPower);
        if (velocity.getMagnitude() > 1) velocity.normalize();

        double xvel = velocity.x * ModuleConstants.MaxSpeed.kLinear;
        double yvel = velocity.y * ModuleConstants.MaxSpeed.kLinear;
        double rvel = rotPower * ModuleConstants.MaxSpeed.kAngular;

        drive(new ChassisSpeeds(xvel, yvel, rvel), fieldRelative);
    }

    /**
     * Field-relative manual swerve drive control.
     *
     * @param xPower   The desired x-direction power. +X is forward, must be [-1, 1]
     * @param yPower   The desired y-direction power. +Y is left, must be [-1, 1]
     * @param rotPower The desired rotational power. +R is ccw, must be [-1, 1]
     */
    public void drive(double xPower, double yPower, double rotPower) {
        drive(xPower, yPower, rotPower, true);
    }

    /**
     * Controller-based swerve drive control
     *
     * @param controller The controller to get input from.
     */
    public void drive(CommandXboxController controller) {
        // [-1..1] inputs w/ deadband
        final double xspeed = MathUtil.applyDeadband(controller.getLeftX(), IOConstants.Controller.kDeadband);
        final double yspeed = MathUtil.applyDeadband(controller.getLeftY(), IOConstants.Controller.kDeadband);
        final double rot = MathUtil.applyDeadband(controller.getRightX(), IOConstants.Controller.kDeadband);

        // do a rate limit
        RateLimiter.Outputs outputs = slew.calculate(xspeed, yspeed, rot);

        // build into a vector with max mag 1 to enforce max speeds correctly
        Vector2 velocity = new Vector2(outputs.xspeed(), outputs.yspeed());
        if (velocity.getMagnitude() > 1) velocity.normalize();

        /*
         * Time to explain some wpilib strangeness
         *
         * xvel, given from the controller, *should* be interpreted as the left-right speed of the robot
         * yvel, given from the controller, *should* be interpreted as the forward-backward speed of the robot
         * however, the WPI coordinate system is such that +Xw is forward, and +Yw is left (using w for WPI)
         * and the controller coordinate system is such that +Xc is right, and +Yc is down (using c for controller)
         * so, we need to mutate x and y, so that +Xc becomes -Yw and +Yc becomes -Xw
         * also, WPIs rotation is ccw-positive and the controller is cw-positive, so we need to negate the rotation
         */
        drive(-velocity.y, -velocity.x, -outputs.rot());
    }

    /**
     * Reset the gyro
     */
    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Add vision measurement
     *
     * @param measurement The vision measurement to add.
     */
    public void addVisionMeasurement(CameraIO.VisionMeasurement measurement) {
        odometry.addVisionMeasurement(measurement.estimate2(), measurement.timestamp());
    }

    @Override
    public void periodic() {
        // update gyro data
        gyro.update();

        // update all modules
        Arrays.stream(modules()).forEach(SwerveIO::update);

        // update odometry
        odometry.update(gyro.heading(), modulePositions());
        Emitter.emit(new PoseUpdateEvent(), getPose());

        // log to akit
        IterUtil.enumerateThen(
            Arrays.stream(modules()), (idx, module) -> {
                final double driveCan = (40 - (idx * 10));
                final String path = "Drivetrain/SwerveModule/" + driveCan;
                Logger.processInputs(path, module.data);
            }
        );

        Logger.recordOutput("Drivetrain/ModuleStates", moduleStates());
        Logger.recordOutput("Drivetrain/ChassisSpeeds", getChassisSpeeds());
        Logger.processInputs("Drivetrain/Gyro", gyro.data);

        SmartDashboard.putData("Field", field);
    }
}
