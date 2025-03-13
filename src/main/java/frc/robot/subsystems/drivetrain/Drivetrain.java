package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.*;
import frc.robot.event.Emitter;
import frc.robot.event.Event;
import frc.robot.event.EventDependency;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.module.ModuleIO;
import frc.robot.subsystems.photon.CameraIO;
import frc.robot.util.IterUtil;
import frc.robot.util.RateLimiter;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.function.Function;
import java.util.stream.Stream;

public class Drivetrain extends SubsystemBase {
    public static class PoseUpdateEvent extends Event<Pose2d> {
        @Override
        public EventDependency<Pose2d, Pose2d> runAfter() {
            return new EventDependency<>(new PoseResetEvent(), Function.identity());
        }
    }

    public static class PoseResetEvent extends Event<Pose2d> { }

    public enum FieldRelativeMode {
        kOff,
        kFixedOrigin,
        kAllianceOrigin,
    }

    final ModuleIO frontLeft;
    final ModuleIO frontRight;
    final ModuleIO rearLeft;
    final ModuleIO rearRight;
    final GyroIO gyro;

    final SwerveDrivePoseEstimator odometry;
    public final Field2d field;

    final RateLimiter slew;

    public Drivetrain(
        GyroIO gyro,
        Function<Integer, ModuleIO> moduleConstructor
    ) {
        this.gyro = gyro;

        frontLeft = moduleConstructor.apply(0);
        frontRight = moduleConstructor.apply(1);
        rearLeft = moduleConstructor.apply(2);
        rearRight = moduleConstructor.apply(3);

        odometry = new SwerveDrivePoseEstimator(
            ModuleConstants.kKinematics,
            gyro.heading(),
            modulePositions(),
            GameConstants.kInitialPose
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
            s -> this.drive(s, FieldRelativeMode.kOff),
            new PPHolonomicDriveController(
                new PIDConstants(
                    ControlConstants.Auto.kOrthoP,
                    ControlConstants.Auto.kOrthoI,
                    ControlConstants.Auto.kOrthoD
                ),
                new PIDConstants(
                    ControlConstants.Auto.kTurnP,
                    ControlConstants.Auto.kTurnI,
                    ControlConstants.Auto.kTurnD
                )
            ),
            new RobotConfig(
                RobotConstants.kMass,
                RobotConstants.kMoI,
                new ModuleConfig(
                    ModuleConstants.Wheel.kRadius,
                    ModuleConstants.MaxSpeed.kLinear,
                    ModuleConstants.Wheel.kFrictionCoefficient,
                    DCMotor.getNEO(1),
                    1.0 / ModuleConstants.DriveMotor.kReduction,
                    MotorConstants.Neo.kCurrentLimit,
                    1
                ),
                ModuleConstants.Translations.kModules
            ),
            () -> DriverStation.getAlliance().map(al -> al == DriverStation.Alliance.Red).orElse(false),
            this
        );

        // Elastic SwerveDrive widget
        SmartDashboard.putData(
            "SwerveDrive",
            builder ->
            {
                builder.setSmartDashboardType("SwerveDrive");

                IterUtil.zipThen(
                    Arrays.stream(modules()),
                    Stream.of("Front Left", "Front Right", "Back Left", "Back Right"),
                    (module, label) ->
                    {
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

                builder.addDoubleProperty(
                    "Robot Angle",
                    () -> {
                        if (GameConstants.kAllianceInvert.get()) {
                            return getHeading().getRadians() + Math.PI;
                        } else {
                            return getHeading().getRadians();
                        }
                    },
                    null
                );
            }
        );

        AutoLogOutputManager.addObject(this);
        Emitter.on(new PoseUpdateEvent(), field::setRobotPose);
        Emitter.on(new PoseResetEvent(), pose -> odometry.resetPosition(gyro.heading(), modulePositions(), pose));
    }

    /**
     * @return A list of all swerve modules on the robot. frontLeft, frontRight, rearLeft, rearRight in that order.
     */
    public ModuleIO[] modules() {
        return new ModuleIO[]{
            frontLeft, frontRight, rearLeft, rearRight
        };
    }

    /**
     * @return A list of all swerve module positions on the robot. In the same order as {@link #modules()}.
     */
    @AutoLogOutput
    public SwerveModulePosition[] modulePositions() {
        return Arrays.stream(modules()).map(ModuleIO::getPosition).toArray(SwerveModulePosition[]::new);
    }

    /**
     * @return A list of all swerve module states on the robot. In the same order as {@link #modules()}.
     */
    public SwerveModuleState[] moduleStates() {
        return Arrays.stream(modules()).map(ModuleIO::getState).toArray(SwerveModuleState[]::new);
    }

    /**
     * @return The speeds of the entire chassis
     */
    public ChassisSpeeds getChassisSpeeds() { return ModuleConstants.kKinematics.toChassisSpeeds(moduleStates()); }

    /**
     * @param speeds The desired speeds for the robot to move at.
     */
    private void setDesiredSpeeds(ChassisSpeeds speeds) {
        // according to delphi, this should remove some skew
        ChassisSpeeds discrete = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states = ModuleConstants.kKinematics.toSwerveModuleStates(discrete);

        // desaturate
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.MaxSpeed.kLinear);

        IterUtil.zipThen(
            Arrays.stream(modules()),
            Arrays.stream(states),
            ModuleIO::setDesiredState
        );
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
    public void drive(ChassisSpeeds speeds, FieldRelativeMode fieldRelative) {
        switch (fieldRelative) {
            case kFixedOrigin -> setDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading()));
            case kAllianceOrigin -> {
                double invert = GameConstants.kAllianceFactor.get();
                setDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds.vxMetersPerSecond * invert,
                    speeds.vyMetersPerSecond * invert,
                    speeds.omegaRadiansPerSecond,
                    getHeading()
                ));
            }
            case kOff -> setDesiredSpeeds(speeds);
        }
    }

    /**
     * Field-relative swerve drive control
     *
     * @param speeds The desired speeds for the robot to move at.
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, FieldRelativeMode.kAllianceOrigin);
    }

    /**
     * Manual swerve drive control
     *
     * @param xPower        The desired x-direction power. +X is forward, must be [-1, 1]
     * @param yPower        The desired y-direction power. +Y is left, must be [-1, 1]
     * @param rotPower      The desired rotational power. +R is ccw, must be [-1, 1]
     * @param fieldRelative Whether the speeds are field-relative or robot-relative. Defaults to true.
     */
    public void drive(double xPower, double yPower, double rotPower, FieldRelativeMode fieldRelative) {
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
        drive(xPower, yPower, rotPower, FieldRelativeMode.kAllianceOrigin);
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
        Emitter.emit(
            new PoseResetEvent(),
            new Pose2d(
                getPose().getTranslation(),
                new Rotation2d()
            )
        );
    }

    /**
     * Add vision measurement
     *
     * @param measurement The vision measurement to add.
     */
    public void addVisionMeasurement(CameraIO.VisionMeasurement measurement) {
        odometry.addVisionMeasurement(measurement.estimate2(), measurement.timestamp(), measurement.stdDevs());
    }

    @Override
    public void periodic() {
        // update gyro data
        gyro.update();

        // update all modules
        Arrays.stream(modules()).forEach(ModuleIO::update);

        // update odometry
        try {
            odometry.update(gyro.heading(), modulePositions());
        } catch (Exception ignored) { }

        Emitter.emit(new PoseUpdateEvent(), getPose());

        // log to akit
        IterUtil.enumerateThen(
            Arrays.stream(modules()),
            (idx, module) ->
            {
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
