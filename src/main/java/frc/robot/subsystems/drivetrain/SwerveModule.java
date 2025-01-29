package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Queue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.constants.ModuleConstants;
import frc.robot.constants.PIDConstants;
import frc.robot.constants.ModuleConstants.DriveEncoder;
import frc.robot.constants.ModuleConstants.TurnEncoder;
import frc.robot.thread.SensorThread;
import frc.robotio.drivetrain.SwerveIO;

public class SwerveModule extends SwerveIO {
	public final SparkMax driveMotor;
	final SparkMax turnMotor;

	final RelativeEncoder driveEncoder;
	final SparkAbsoluteEncoder turnEncoder;

	final SparkClosedLoopController driveController;
	final SparkClosedLoopController turnController;

	final Queue<Distance> driveCache;
	final Queue<Angle> turnCache;
	final Queue<Double> timestamps;

	SwerveModuleState desiredState;

	public SwerveModule(
		int driveId,
		int turnId,
		double angularOffset
	) {
		super(driveId, turnId, angularOffset);

		// initialize motors, encoders, etc.
		driveMotor = new SparkMax(driveId, MotorType.kBrushless);
		turnMotor = new SparkMax(turnId, MotorType.kBrushless);

		driveEncoder = driveMotor.getEncoder();
		turnEncoder = turnMotor.getAbsoluteEncoder();

		driveController = driveMotor.getClosedLoopController();
		turnController = turnMotor.getClosedLoopController();

		desiredState = new SwerveModuleState();

		// configure motors
		SparkMaxConfig driveConfig = new SparkMaxConfig();
		SparkMaxConfig turnConfig = new SparkMaxConfig();

		// position, velocity factors
		driveConfig.encoder.positionConversionFactor(DriveEncoder.kPositionFactor);
		driveConfig.encoder.velocityConversionFactor(DriveEncoder.kVelocityFactor);
		turnConfig.absoluteEncoder.positionConversionFactor(TurnEncoder.kPositionFactor);
		turnConfig.absoluteEncoder.velocityConversionFactor(TurnEncoder.kVelocityFactor);

		// configure feedback encoders
		turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

		// invert encoders
		turnConfig.absoluteEncoder.inverted(TurnEncoder.kInverted);

		// pid wrapping
		turnConfig.closedLoop.positionWrappingEnabled(true);
		turnConfig.closedLoop.positionWrappingMinInput(TurnEncoder.kMinPidInput);
		turnConfig.closedLoop.positionWrappingMaxInput(TurnEncoder.kMaxPidInput);

		// pid constants
		driveConfig.closedLoop.pidf(
			PIDConstants.DriveMotor.kP,
			PIDConstants.DriveMotor.kI,
			PIDConstants.DriveMotor.kD,
			PIDConstants.DriveMotor.kF
		);

		driveConfig.closedLoop.outputRange(
			PIDConstants.DriveMotor.kMin,
			PIDConstants.DriveMotor.kMax
		);

		turnConfig.closedLoop.pidf(
			PIDConstants.TurnMotor.kP,
			PIDConstants.TurnMotor.kI,
			PIDConstants.TurnMotor.kD,
			PIDConstants.TurnMotor.kF
		);

		turnConfig.closedLoop.outputRange(
			PIDConstants.TurnMotor.kMin,
			PIDConstants.TurnMotor.kMax
		);

		// misc
		driveConfig.idleMode(ModuleConstants.Neo.kIdleMode);
		turnConfig.idleMode(ModuleConstants.Neo.kIdleMode);
		driveConfig.smartCurrentLimit((int) ModuleConstants.Neo.kCurrentLimit);
		turnConfig.smartCurrentLimit((int) ModuleConstants.Neo.kCurrentLimit);

		// apply and burn configs
		driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// initialize caches
		driveCache = SensorThread.getInstance().register(() -> Meters.of(driveEncoder.getPosition()));
		turnCache = SensorThread.getInstance().register(() -> Radians.of(turnEncoder.getPosition()));
		timestamps = SensorThread.getInstance().addTimestamps();
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

	@Override
	public void update() {
		this.data.driveConnected = driveMotor.getLastError() != REVLibError.kOk;
		this.data.drivePosition = Meters.of(driveEncoder.getPosition());
		this.data.driveVelocity = MetersPerSecond.of(driveEncoder.getVelocity());
		this.data.driveVoltage = Volts.of(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput());
		this.data.driveCurrent = Amps.of(driveMotor.getOutputCurrent());

		this.data.turnConnected = turnMotor.getLastError() != REVLibError.kOk;
		this.data.turnPosition = Radians.of(turnEncoder.getPosition());
		this.data.turnVelocity = RadiansPerSecond.of(turnEncoder.getVelocity());
		this.data.turnVoltage = Volts.of(turnMotor.getBusVoltage() * turnMotor.getAppliedOutput());
		this.data.turnCurrent = Amps.of(turnMotor.getOutputCurrent());

		driveCache.clear();
		turnCache.clear();
	}
}

