package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.*;
import frc.robot.event.EventRegistry;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.module.ModuleIO;
import frc.robot.subsystems.photon.CameraIO;
import frc.robot.util.AdvantageUtil;
import frc.robot.util.AllianceUtil;
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
    final Field2d field;

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

        this.gyro.reset(new Rotation2d());

        AutoBuilder.configure(
            this::getPose,
            EventRegistry.poseReset::emit,
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
            AllianceUtil::invert,
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
                    () -> AllianceUtil.recenter(getHeading()).getRadians(),
                    null
                );
            }
        );

        // template code stuff
        HAL.report(FRCNetComm.tResourceType.kResourceType_RobotDrive, FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit);
        AutoLogOutputManager.addObject(this);

        // setup event emitter stuff
        EventRegistry.poseUpdate.register(field::setRobotPose);
        EventRegistry.poseReset.register(p -> odometry.resetPosition(gyro.heading(), modulePositions(), p));
    }

    /**
     * @return A list of all swerve modules on the robot. frontLeft, frontRight, rearLeft, rearRight in that order.
     */
    public ModuleIO[] modules() {
        return new ModuleIO[]{ frontLeft, frontRight, rearLeft, rearRight };
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
    @AutoLogOutput
    public SwerveModuleState[] moduleStates() {
        return Arrays.stream(modules()).map(ModuleIO::getState).toArray(SwerveModuleState[]::new);
    }

    /** @return The speeds of the entire chassis */
    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds() { return ModuleConstants.kKinematics.toChassisSpeeds(moduleStates()); }

    /** @param speeds The desired speeds for the robot to move at */
    private void setDesiredSpeeds(ChassisSpeeds speeds) {
        // according to delphi, this should remove some skew
        ChassisSpeeds discrete = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states = ModuleConstants.kKinematics.toSwerveModuleStates(discrete);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.MaxSpeed.kLinear);

        Logger.recordOutput("Drivetrain/StateSetpoints", states);
        Logger.recordOutput("Drivetrain/SpeedSetpoint", discrete);

        IterUtil.zipThen(Arrays.stream(modules()), Arrays.stream(states), ModuleIO::setDesiredState);
    }

    /** @return The estimated pose of the robot */
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
            case kAllianceOrigin -> setDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond * AllianceUtil.factor(),
                speeds.vyMetersPerSecond * AllianceUtil.factor(),
                speeds.omegaRadiansPerSecond,
                getHeading()
            ));
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
        double xspeed = MathUtil.applyDeadband(controller.getLeftX(), IOConstants.Controller.kDeadband);
        double yspeed = MathUtil.applyDeadband(controller.getLeftY(), IOConstants.Controller.kDeadband);
        double rot = MathUtil.applyDeadband(controller.getRightX(), IOConstants.Controller.kDeadband);

        // do a rate limit
        RateLimiter.Outputs outputs = slew.calculate(xspeed, yspeed, rot);

        // build into a vector with max mag 1 to enforce max speeds correctly
        Vector2 velocity = new Vector2(outputs.xspeed(), outputs.yspeed());
        if (velocity.getMagnitude() > 1) velocity.normalize();
        Vector2 velocity2 = velocity.multiply(velocity.getMagnitudeSquared());

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
        drive(-velocity2.y, -velocity2.x, -outputs.rot());
    }

    /**
     * Reset the gyro
     */
    public void resetGyro() {
        Rotation2d reset = AllianceUtil.facingDriver();
        gyro.reset(reset);
        EventRegistry.poseReset.emit(new Pose2d(getPose().getTranslation(), reset));
    }

    /**
     * Add vision measurement
     *
     * @param measurement The vision measurement to add.
     */
    public void addVisionMeasurement(CameraIO.VisionMeasurement measurement) {
        odometry.addVisionMeasurement(measurement.estimate2(), measurement.timestamp(), measurement.stdDevs());
    }

    /**
     * Get Field widget
     */
    public Field2d getField() {
        return field;
    }

    @Override
    public void periodic() {
        // update gyro data
        gyro.update();

        // update all modules
        Arrays.stream(modules()).forEach(ModuleIO::update);

        // get heading, use robot-centric as fallback
        Rotation2d heading;

        if (gyro.data.connected) heading = gyro.heading();
        else heading = AllianceUtil.recenter(new Rotation2d());

        // update odometry
        try { odometry.update(heading, modulePositions()); } catch (Exception ignored) { }

        EventRegistry.poseUpdate.emit(getPose());

        // log to akit
        IterUtil.enumerateThen(
            Arrays.stream(modules()),
            (idx, module) ->
            {
                final String path = "Drivetrain/SwerveModule/" + module.moduleId;
                Logger.processInputs(path, module.data);
            }
        );

        Logger.recordOutput("Drivetrain/ModuleStates", moduleStates());
        Logger.recordOutput("Drivetrain/ChassisSpeeds", getChassisSpeeds());
        Logger.processInputs("Drivetrain/Gyro", gyro.data);
        AdvantageUtil.logActiveCommand(this);

        SmartDashboard.putData("Field", field);
    }
}
