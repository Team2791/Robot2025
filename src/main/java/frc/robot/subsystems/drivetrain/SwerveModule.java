package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.ModuleConstants;
import frc.robot.constants.PIDConstants;

public class SwerveModule {
	final SparkMax driveMotor;
	final SparkMax turnMotor;

	final RelativeEncoder driveEncoder;
	final SparkAbsoluteEncoder turnEncoder;

	final SparkClosedLoopController driveController;
	final SparkClosedLoopController turnController;

	SwerveModuleState desiredState;

	final Angle angularOffset;

	public SwerveModule(
		int driveId,
		int turnId,
		Angle angularOffset
	) {
		// initialize motors, encoders, etc.
		driveMotor = new SparkMax(driveId, MotorType.kBrushless);
		turnMotor = new SparkMax(turnId, MotorType.kBrushless);

		driveEncoder = driveMotor.getEncoder();
		turnEncoder = turnMotor.getAbsoluteEncoder();

		driveController = driveMotor.getClosedLoopController();
		turnController = turnMotor.getClosedLoopController();

		desiredState = new SwerveModuleState();

		this.angularOffset = angularOffset;

		// configure motors
		SparkMaxConfig driveConfig = new SparkMaxConfig();
		SparkMaxConfig turnConfig = new SparkMaxConfig();

		// position, velocity factors
		driveConfig.encoder.positionConversionFactor(ModuleConstants.DriveEncoder.kPositionFactor);
		driveConfig.encoder.velocityConversionFactor(ModuleConstants.DriveEncoder.kVelocityFactor);
		turnConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.TurnEncoder.kPositionFactor);
		turnConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.TurnEncoder.kVelocityFactor);

		// invert turn encoder
		turnConfig.absoluteEncoder.inverted(ModuleConstants.TurnEncoder.kInverted);

		// pid wrapping
		turnConfig.closedLoop.positionWrappingEnabled(true);
		turnConfig.closedLoop.positionWrappingMinInput(ModuleConstants.TurnEncoder.kMinPidInput);
		turnConfig.closedLoop.positionWrappingMaxInput(ModuleConstants.TurnEncoder.kMaxPidInput);

		// pid constants
		driveConfig.closedLoop.pidf(
			PIDConstants.DriveMotor.kP,
			PIDConstants.DriveMotor.kI,
			PIDConstants.DriveMotor.kD,
			PIDConstants.DriveMotor.kFF
		);

		driveConfig.closedLoop.outputRange(
			PIDConstants.DriveMotor.kMin,
			PIDConstants.DriveMotor.kMax
		);

		turnConfig.closedLoop.pidf(
			PIDConstants.TurnMotor.kP,
			PIDConstants.TurnMotor.kI,
			PIDConstants.TurnMotor.kD,
			PIDConstants.TurnMotor.kFF
		);

		turnConfig.closedLoop.outputRange(
			PIDConstants.TurnMotor.kMin,
			PIDConstants.TurnMotor.kMax
		);

		// misc
		driveConfig.idleMode(ModuleConstants.Spark.kIdleMode);
		turnConfig.idleMode(ModuleConstants.Spark.kIdleMode);
		driveConfig.smartCurrentLimit((int) ModuleConstants.Spark.kCurrentLimit.in(Amps));
		turnConfig.smartCurrentLimit((int) ModuleConstants.Spark.kCurrentLimit.in(Amps));

		// apply and burn configs
		driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
		turnMotor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
	}

	public void setDesiredState(SwerveModuleState desired) {
		SwerveModuleState corrected = new SwerveModuleState(
			desired.speedMetersPerSecond,
			desired.angle.plus(new Rotation2d(angularOffset))
		);

		corrected.optimize(new Rotation2d(turnEncoder.getPosition()));

		driveController.setReference(corrected.speedMetersPerSecond, ControlType.kVelocity);
		turnController.setReference(corrected.angle.getRadians(), ControlType.kPosition);

		desiredState = corrected;
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			driveEncoder.getPosition(),
			new Rotation2d(turnEncoder.getPosition() - angularOffset.in(Radians))
		);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			driveEncoder.getVelocity(),
			new Rotation2d(turnEncoder.getPosition())
		);
	}
}

