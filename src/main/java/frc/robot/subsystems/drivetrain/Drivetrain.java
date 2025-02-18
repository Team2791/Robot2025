package frc.robot.subsystems.drivetrain;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.AdvantageConstants;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.PIDConstants;
import frc.robot.constants.AdvantageConstants.AdvantageMode;
import frc.robot.util.IterUtil;
import frc.robot.util.TriSupplier;
import frc.robotio.drivetrain.GyroIO;
import frc.robotio.drivetrain.SwerveIO;
import frc.robotsim.drivetrain.GyroSim;

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
		TriSupplier<Integer, Integer, Double, SwerveIO> moduleConstructor
	) {
		this.gyro = gyro;

		frontLeft = moduleConstructor.get(
			IOConstants.Drivetrain.Drive.kFrontLeft,
			IOConstants.Drivetrain.Turn.kFrontLeft,
			DriveConstants.AngularOffsets.kFrontLeft
		);

		frontRight = moduleConstructor.get(
			IOConstants.Drivetrain.Drive.kFrontRight,
			IOConstants.Drivetrain.Turn.kFrontRight,
			DriveConstants.AngularOffsets.kFrontRight
		);

		rearLeft = moduleConstructor.get(
			IOConstants.Drivetrain.Drive.kRearLeft,
			IOConstants.Drivetrain.Turn.kRearLeft,
			DriveConstants.AngularOffsets.kRearLeft
		);

		rearRight = moduleConstructor.get(
			IOConstants.Drivetrain.Drive.kRearRight,
			IOConstants.Drivetrain.Turn.kRearRight,
			DriveConstants.AngularOffsets.kRearRight
		);

		odometry = new SwerveDrivePoseEstimator(
			DriveConstants.kKinematics,
			gyro.heading(),
			modulePositions(),
			new Pose2d()
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
	public List<SwerveModuleState> moduleStates() {
		return Arrays.stream(modules()).map(SwerveIO::getState).toList();
	}

	/**
	 * @return The speeds of all swerve modules on the robot. In the same order as {@link #modules()}.
	 */
	public ChassisSpeeds getChassisSpeeds() {
		return DriveConstants.kKinematics.toChassisSpeeds(moduleStates().toArray(SwerveModuleState[]::new));
	}

	/**
	 * @param speeds The desired speeds for the robot to move at.
	 */
	public void setDesiredSpeeds(ChassisSpeeds speeds) {
		SwerveModuleState[] states = DriveConstants.kKinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeed);

		// set the desired states of all modules. i miss kotlin :(
		IterUtil.zipThen(Arrays.stream(modules()), Arrays.stream(states), SwerveIO::setDesiredState);

		// simulation gyro needs this
		if (AdvantageConstants.Modes.kCurrent == AdvantageMode.Sim) {
			((GyroSim) gyro).updateOmega(speeds.omegaRadiansPerSecond);
		}
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
		drive(speeds, false);
	}

	/**
	 * Manual swerve drive control
	 * 
	 * @param xspeed        The desired speed for the robot to move in the x direction.
	 * @param yspeed        The desired speed for the robot to move in the y direction.
	 * @param rot           The desired rotation for the robot to move with.
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
	 * @param rot    The desired rotation for the robot to move with.
	 */
	public void drive(double xspeed, double yspeed, double rot) {
		drive(xspeed, yspeed, rot, false);
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
		// TODO: SlewWrapper.SlewOutputs slewOutputs = slew.update(xspeed, yspeed, rot);

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

		Logger.processInputs("Drivetrain/Gyro", gyro.data);
	}
}
