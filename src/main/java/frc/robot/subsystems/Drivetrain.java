package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Arrays;
import java.util.List;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DriveConstants;
import frc.robot.helpers.StreamHelpers;
import frc.robot.swerve.IOConstants;
import frc.robot.swerve.ServeUtil;
import frc.robot.swerve.SwerveModule;

public class Drivetrain extends SubsystemBase {
	final SwerveModule frontLeft;
	final SwerveModule frontRight;
	final SwerveModule rearLeft;
	final SwerveModule rearRight;

	final AHRS gyro;
	final SwerveDrivePoseEstimator odometry;
	final Field2d field;

	final SlewRateLimiter magSlew = new SlewRateLimiter(DriveConstants.Slew.kMagnitude);
	final SlewRateLimiter rotSlew = new SlewRateLimiter(DriveConstants.Slew.kRotation);

	/**
	 * @return The drivetrain's heading
	 */
	public Rotation2d getHeading() {
		return Rotation2d.fromDegrees(gyro.getAngle() * DriveConstants.kGyroFactor); // use kGyroFactor to invert the gyro
	}

	/**
	 * @return The gyro's reported heading, without modifications.
	 */
	public Rotation2d gyroRaw() {
		return Rotation2d.fromDegrees(gyro.getAngle());
	}

	/**
	 * @return A list of all swerve modules on the robot. frontLeft, frontRight, rearLeft, rearRight in that order.
	 */
	public List<SwerveModule> modules() {
		return List.of(frontLeft, frontRight, rearLeft, rearRight);
	}

	/**
	 * @return A list of all swerve module positions on the robot. In the same order as {@link #modules()}.
	 */
	public List<SwerveModulePosition> modulePositions() {
		return modules().stream().map(SwerveModule::getPosition).toList();
	}

	/**
	 * @return A list of all swerve module states on the robot. In the same order as {@link #modules()}.
	 */
	public List<SwerveModuleState> moduleStates() {
		return modules().stream().map(SwerveModule::getState).toList();
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
		StreamHelpers.zipThen(modules().stream(), Arrays.stream(states), SwerveModule::setDesiredState);
	}

	/**
	 * @return The estimated pose of the robot.
	 */
	public Pose2d getPose() { return odometry.getEstimatedPosition(); }

	/**
	 * @param pose The new pose of the robot.
	 */
	private void resetOdometry(Pose2d pose) {
		odometry.resetPosition(getHeading(), modulePositions().toArray(SwerveModulePosition[]::new), pose);
	}

	// TODO: auto stuff
	public Drivetrain() {
		frontLeft = new SwerveModule(
			IOConstants.Drivetrain.Drive.kFrontLeft,
			IOConstants.Drivetrain.Turn.kFrontLeft,
			DriveConstants.AngularOffsets.kFrontLeft
		);

		frontRight = new SwerveModule(
			IOConstants.Drivetrain.Drive.kFrontRight,
			IOConstants.Drivetrain.Turn.kFrontRight,
			DriveConstants.AngularOffsets.kFrontRight
		);

		rearLeft = new SwerveModule(
			IOConstants.Drivetrain.Drive.kRearLeft,
			IOConstants.Drivetrain.Turn.kRearLeft,
			DriveConstants.AngularOffsets.kRearLeft
		);

		rearRight = new SwerveModule(
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

		gyro = new AHRS(NavXComType.kMXP_SPI);
		field = new Field2d();

		gyro.reset();

		ShuffleboardTab drive = Shuffleboard.getTab("Drivetrain");

		drive.addNumber("Heading (degrees)", () -> getHeading().getDegrees());
		drive.addNumber("X Speed", () -> getChassisSpeeds().vxMetersPerSecond);
		drive.addNumber("Y Speed", () -> getChassisSpeeds().vyMetersPerSecond);
		drive.addNumber("Angular Speed", () -> getChassisSpeeds().omegaRadiansPerSecond);
		drive.add("Field", field);
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
}
