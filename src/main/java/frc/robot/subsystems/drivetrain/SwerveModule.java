package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Queue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;

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

import frc.robot.constants.IOConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.PIDConstants;
import frc.robot.constants.ModuleConstants.DriveEncoder;
import frc.robot.constants.ModuleConstants.TurnEncoder;
import frc.robot.thread.SensorThread;
import frc.robot.util.IterUtil;
import frc.robotio.drivetrain.SwerveIO;

public class SwerveModule extends SwerveIO {
	final SparkMax driveMotor;
	final SparkMax turnMotor;

	final RelativeEncoder driveEncoder;
	final SparkAbsoluteEncoder turnEncoder;

	final SparkClosedLoopController driveController;
	final SparkClosedLoopController turnController;

	final Queue<Angle> driveHist;
	final Queue<Angle> turnHist;
	final Queue<Double> timestamps;

	final double angularOffset;

	SwerveModuleState desiredState;

	/**
	 * Constructs a new SwerveModule
	 * 
	 * @param id the module's id. [0..=3] corresponds to [fl, fr, bl, br]
	 */
	public SwerveModule(int id) {
		int driveId = switch (id) {
			case 0 -> IOConstants.Drivetrain.Drive.kFrontLeft;
			case 1 -> IOConstants.Drivetrain.Drive.kFrontRight;
			case 2 -> IOConstants.Drivetrain.Drive.kRearLeft;
			case 3 -> IOConstants.Drivetrain.Drive.kRearRight;
			default -> throw new IllegalArgumentException("Invalid module id: " + id);
		};

		int turnId = switch (id) {
			case 0 -> IOConstants.Drivetrain.Turn.kFrontLeft;
			case 1 -> IOConstants.Drivetrain.Turn.kFrontRight;
			case 2 -> IOConstants.Drivetrain.Turn.kRearLeft;
			case 3 -> IOConstants.Drivetrain.Turn.kRearRight;
			default -> throw new IllegalArgumentException("Invalid module id: " + id);
		};

		angularOffset = switch (id) {
			case 0 -> ModuleConstants.AngularOffsets.kFrontLeft;
			case 1 -> ModuleConstants.AngularOffsets.kFrontRight;
			case 2 -> ModuleConstants.AngularOffsets.kRearLeft;
			case 3 -> ModuleConstants.AngularOffsets.kRearRight;
			default -> throw new IllegalArgumentException("Invalid module id: " + id);
		};

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

		// invert encoders
		turnConfig.absoluteEncoder.inverted(TurnEncoder.kInverted);
		turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

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
		turnConfig.idleMode(ModuleConstants.Neo550.kIdleMode);
		driveConfig.smartCurrentLimit((int) ModuleConstants.Neo.kCurrentLimit);
		turnConfig.smartCurrentLimit((int) ModuleConstants.Neo550.kCurrentLimit);

		// apply and burn configs
		driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// initialize caches
		driveHist = SensorThread.getInstance().register(() -> Radians.of(driveEncoder.getPosition()));
		turnHist = SensorThread.getInstance().register(() -> Radians.of(turnEncoder.getPosition()));
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

		this.data.desired = corrected;
	}

	@Override
	public void update() {
		this.data.driveConnected = driveMotor.getLastError() != REVLibError.kOk;
		this.data.drivePosition = Radians.of(driveEncoder.getPosition());
		this.data.driveVelocity = RadiansPerSecond.of(driveEncoder.getVelocity());
		this.data.driveVoltage = Volts.of(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput());
		this.data.driveCurrent = Amps.of(driveMotor.getOutputCurrent());

		this.data.turnConnected = turnMotor.getLastError() != REVLibError.kOk;
		this.data.turnPosition = Radians.of(turnEncoder.getPosition() - angularOffset);
		this.data.turnVelocity = RadiansPerSecond.of(turnEncoder.getVelocity());
		this.data.turnVoltage = Volts.of(turnMotor.getBusVoltage() * turnMotor.getAppliedOutput());
		this.data.turnCurrent = Amps.of(turnMotor.getOutputCurrent());

		this.data.timestamps = IterUtil.toDoubleArray(timestamps.stream());
		this.data.drivePositions = IterUtil.toDoubleArray(driveHist.stream(), Radians);
		this.data.turnPositions = IterUtil.toDoubleArray(turnHist.stream(), Radians);

		driveHist.clear();
		turnHist.clear();
	}
}

