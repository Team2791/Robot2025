package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.util.StreamUtil;
import frc.robot.util.TriSupplier;
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

	/**
	 * @return The drivetrain's heading
	 */
	public Rotation2d getHeading() { return new Rotation2d(gyro.data.heading); }

	/**
	 * @return A list of all swerve modules on the robot. frontLeft, frontRight, rearLeft, rearRight in that order.
	 */
	public List<SwerveIO> modules() {
		return List.of(frontLeft, frontRight, rearLeft, rearRight);
	}

	/**
	 * @return A list of all swerve module positions on the robot. In the same order as {@link #modules()}.
	 */
	public List<SwerveModulePosition> modulePositions() {
		return modules().stream().map(SwerveIO::getPosition).toList();
	}

	/**
	 * @return A list of all swerve module states on the robot. In the same order as {@link #modules()}.
	 */
	public List<SwerveModuleState> moduleStates() {
		return modules().stream().map(SwerveIO::getState).toList();
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
		SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeed.in(MetersPerSecond));

		// set the desired states of all modules. i miss kotlin :(
		StreamUtil.zipThen(modules().stream(), Arrays.stream(states), SwerveIO::setDesiredState);
	}

	/**
	 * @return The estimated pose of the robot.
	 */
	@AutoLogOutput
	public Pose2d getPose() { return odometry.getEstimatedPosition(); }

	/**
	 * @param pose The new pose of the robot.
	 */
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(getHeading(), modulePositions().toArray(SwerveModulePosition[]::new), pose);
	}

	// TODO: auto stuff
	public Drivetrain(
		GyroIO gyro,
		TriSupplier<Integer, Integer, Angle, SwerveIO> moduleConstructor
	) {
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
			getHeading(),
			modulePositions().toArray(SwerveModulePosition[]::new),
			new Pose2d()
		);

		slew = new SlewWrapper(
			DriveConstants.Slew.kMagnitude,
			DriveConstants.Slew.kRotation,
			DriveConstants.Slew.kDirection
		);

		this.gyro = gyro;
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
	 * @param rot           The desired rotation for the robot to move with.
	 * @param fieldRelative Whether the speeds are field-relative or robot-relative. Defaults to true.
	 */
	public void drive(LinearVelocity xspeed, LinearVelocity yspeed, AngularVelocity rot, boolean fieldRelative) {
		drive(new ChassisSpeeds(xspeed, yspeed, rot), fieldRelative);
	}

	/**
	 * Field-relative manual swerve drive control
	 * 
	 * @param xspeed The desired speed for the robot to move in the x direction.
	 * @param yspeed The desired speed for the robot to move in the y direction.
	 * @param rot    The desired rotation for the robot to move with.
	 */
	public void drive(LinearVelocity xspeed, LinearVelocity yspeed, AngularVelocity rot) {
		drive(xspeed, yspeed, rot, true);
	}

	/**
	 * Controller-based swerve drive control
	 * 
	 * @param controller The controller to get input from.
	 */
	public void drive(CommandXboxController controller) {
		// [-1..1] inputs w/ deadband
		double xspeed = MathUtil.applyDeadband(controller.getLeftX(), ControllerConstants.kDeadband);
		double yspeed = MathUtil.applyDeadband(controller.getLeftY(), ControllerConstants.kDeadband);
		double rot = MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDeadband);

		// do a rate limit
		SlewWrapper.SlewOutputs slewOutputs = slew.update(xspeed, yspeed, rot);

		// multiply by max speed
		LinearVelocity xvel = DriveConstants.MaxSpeed.kLinear.times(slewOutputs.xspeed);
		LinearVelocity yvel = DriveConstants.MaxSpeed.kLinear.times(slewOutputs.yspeed);
		AngularVelocity rvel = DriveConstants.MaxSpeed.kAngular.times(slewOutputs.rot);

		drive(xvel, yvel, rvel);
	}

	@Override
	public void periodic() {
		// update gyro data
		gyro.update();

		// update all modules
		modules().forEach(SwerveIO::update);

		// update odometry
		odometry.update(getHeading(), modulePositions().toArray(SwerveModulePosition[]::new));
		field.setRobotPose(getPose());

		// log to advantagekit
		StreamUtil.enumerateThen(modules().stream(), (idx, module) -> {
			final double driveCan = (40 - (idx * 10));
			final String path = "Drivetrain/SwerveModule/" + driveCan;
			Logger.processInputs(path, module.data);
		});

		Logger.processInputs("Drivetrain/Gyro", gyro.data);
	}
}
