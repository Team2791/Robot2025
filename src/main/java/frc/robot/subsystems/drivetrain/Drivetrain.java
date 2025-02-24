package frc.robot.subsystems.drivetrain;

import java.io.IOException;
import java.util.Arrays;
import java.util.function.Function;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PIDConstants;
import frc.robot.maple.MapleSim;
import frc.robot.util.IterUtil;
import frc.robotio.drivetrain.GyroIO;
import frc.robotio.drivetrain.SwerveIO;

public class Drivetrain extends SubsystemBase {
	final SwerveIO frontLeft;
	final SwerveIO frontRight;
	final SwerveIO rearLeft;
	final SwerveIO rearRight;

	final GyroIO gyro;

	final SwerveDrivePoseEstimator odometry;
	final Field2d field;

	final SlewWrapper slew;

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
			new Pose2d(7.5, 5, new Rotation2d())
		);

		slew = new SlewWrapper(
			DriveConstants.Slew.kMagnitude,
			DriveConstants.Slew.kRotation,
			DriveConstants.Slew.kDirection
		);

		field = new Field2d();

		this.gyro.reset();

		ShuffleboardTab drive = Shuffleboard.getTab("Drivetrain");

		drive.addNumber("Heading (degrees)", () -> getHeading().getDegrees());
		drive.addNumber("X Speed", () -> getChassisSpeeds().vxMetersPerSecond);
		drive.addNumber("Y Speed", () -> getChassisSpeeds().vyMetersPerSecond);
		drive.addNumber("Angular Speed", () -> getChassisSpeeds().omegaRadiansPerSecond);
		drive.add("Field", field);

		AutoBuilder.configure(
			this::getPose,
			this::resetOdometry,
			this::getChassisSpeeds,
			s -> this.drive(s, false),
			new PPHolonomicDriveController(
				new com.pathplanner.lib.config.PIDConstants(
					PIDConstants.Autos.kOrthoP,
					PIDConstants.Autos.kOrthoI,
					PIDConstants.Autos.kOrthoD
				),
				new com.pathplanner.lib.config.PIDConstants(
					PIDConstants.Autos.kTurnP,
					PIDConstants.Autos.kTurnI,
					PIDConstants.Autos.kTurnD
				)
			),
			RobotConfig.fromGUISettings(),
			() -> DriverStation.getAlliance().map(al -> al == DriverStation.Alliance.Red).orElse(false),
			this
		);

		PathPlannerLogging.setLogActivePathCallback(
			(path) -> Logger.recordOutput("Autos/Path", path.toArray(Pose2d[]::new))
		);
		PathPlannerLogging.setLogTargetPoseCallback(
			(pose) -> Logger.recordOutput("Autos/TargetPose", pose)
		);

		AutoLogOutputManager.addObject(this);
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
	public void setDesiredSpeeds(ChassisSpeeds speeds) {
		ChassisSpeeds descrete = ChassisSpeeds.discretize(speeds, 0.02);
		SwerveModuleState[] states = DriveConstants.kKinematics.toSwerveModuleStates(descrete);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MaxSpeed.kLinear);

		// set the desired states of all modules. i miss kotlin :(
		IterUtil.zipThen(Arrays.stream(modules()), Arrays.stream(states), SwerveIO::setDesiredState);
	}

	/**
	 * @return The estimated pose of the robot.
	 */
	@AutoLogOutput
	public Pose2d getPose() { return odometry.getEstimatedPosition(); }

	public Rotation2d getHeading() { return getPose().getRotation(); }

	/**
	 * @param pose The new pose of the robot.
	 */
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(gyro.heading(), modulePositions(), pose);
		MapleSim.getInstance().resetPose(pose);
	}

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
	 * @param xspeed        The desired speed for the robot to move in the x direction.
	 * @param yspeed        The desired speed for the robot to move in the y direction.
	 * @param rot           The desired rotational speed
	 * @param fieldRelative Whether the speeds are field-relative or robot-relative. Defaults to true.
	 */
	public void drive(double xspeed, double yspeed, double rot, boolean fieldRelative) {
		drive(new ChassisSpeeds(xspeed, yspeed, rot), fieldRelative);
	}

	/**
	 * Field-relative manual swerve drive control
	 * 
	 * @param xspeed The desired speed for the robot to move in the x direction.
	 * @param yspeed The desired speed for the robot to move in the y direction.
	 * @param rot    The desired rotational speed
	 */
	public void drive(double xspeed, double yspeed, double rot) {
		drive(xspeed, yspeed, rot, true);
	}

	/**
	 * Controller-based swerve drive control
	 * 
	 * @param controller The controller to get input from.
	 */
	public void drive(CommandXboxController controller) {
		// [-1..1] inputs w/ deadband
		final double xspeed = MathUtil.applyDeadband(controller.getLeftX(), ControllerConstants.kDeadband);
		final double yspeed = MathUtil.applyDeadband(controller.getLeftY(), ControllerConstants.kDeadband);
		final double rot = MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDeadband);

		// do a rate limit
		// SlewWrapper.SlewOutputs slewOutputs = slew.update(xspeed, yspeed, rot);

		// multiply by max speed
		double xvel = DriveConstants.MaxSpeed.kLinear * xspeed;
		double yvel = DriveConstants.MaxSpeed.kLinear * yspeed;
		double rvel = DriveConstants.MaxSpeed.kAngular * rot;

		drive(xvel, yvel, rvel);
	}

	@Override
	public void periodic() {
		// update gyro data
		gyro.update();

		// update all modules
		Arrays.stream(modules()).forEach(SwerveIO::update);

		// update odometry
		odometry.update(gyro.heading(), modulePositions());
		field.setRobotPose(getPose());

		// log to advantagekit
		IterUtil.enumerateThen(Arrays.stream(modules()), (idx, module) -> {
			final double driveCan = (40 - (idx * 10));
			final String path = "Drivetrain/SwerveModule/" + driveCan;
			Logger.processInputs(path, module.data);
		});

		Logger.recordOutput("Drivetrain/ModuleStates", moduleStates());
		Logger.recordOutput("Drivetrain/ChassisSpeeds", getChassisSpeeds());
		Logger.processInputs("Drivetrain/Gyro", gyro.data);
	}
}
