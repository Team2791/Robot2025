package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Queue;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.IOConstants;
import frc.robot.constants.MotorConstants;
import frc.robot.constants.PIDConstants;
import frc.robotio.scoring.ElevatorIO;
import frc.robot.constants.ElevatorConstants.Encoder;
import frc.robot.constants.ElevatorConstants.Motor;
import frc.robot.thread.SensorThread;
import frc.robot.util.IterUtil;

public class Elevator extends ElevatorIO {
	final SparkFlex leader;
	final SparkFlex follower;

	final RelativeEncoder encoder;
	final SparkClosedLoopController controller;

	final Queue<Angle> positionHist;
	final Queue<Double> timestamps;

	public Elevator() {
		leader = new SparkFlex(IOConstants.Elevator.kLeader, MotorType.kBrushless);
		follower = new SparkFlex(IOConstants.Elevator.kFollower, MotorType.kBrushless);

		encoder = leader.getEncoder();
		controller = leader.getClosedLoopController();

		final SparkFlexConfig leaderConfig = new SparkFlexConfig();
		final SparkFlexConfig followerConfig = new SparkFlexConfig();

		// current limits
		leaderConfig.smartCurrentLimit((int) MotorConstants.Neo.kCurrentLimit);
		followerConfig.smartCurrentLimit((int) MotorConstants.Neo.kCurrentLimit);

		// position and velocity factors
		leaderConfig.encoder.positionConversionFactor(Encoder.kPositionFactor);
		followerConfig.encoder.positionConversionFactor(Encoder.kPositionFactor);
		leaderConfig.encoder.velocityConversionFactor(Encoder.kVelocityFactor);
		followerConfig.encoder.velocityConversionFactor(Encoder.kVelocityFactor);

		// encoder config
		leaderConfig.encoder.inverted(Encoder.kInverted);
		followerConfig.encoder.inverted(Encoder.kInverted);

		// pid constants
		leaderConfig.closedLoop.pidf(
			PIDConstants.Elevator.kP,
			PIDConstants.Elevator.kI,
			PIDConstants.Elevator.kD,
			PIDConstants.Elevator.kF
		);

		leaderConfig.closedLoop.outputRange(
			PIDConstants.Elevator.kMin,
			PIDConstants.Elevator.kMax
		);

		followerConfig.closedLoop.pidf(
			PIDConstants.Elevator.kP,
			PIDConstants.Elevator.kI,
			PIDConstants.Elevator.kD,
			PIDConstants.Elevator.kF
		);

		followerConfig.closedLoop.outputRange(
			PIDConstants.Elevator.kMin,
			PIDConstants.Elevator.kMax
		);

		// misc
		leaderConfig.idleMode(Motor.kIdleMode);
		followerConfig.idleMode(Motor.kIdleMode);
		followerConfig.follow(leader);

		// apply configs
		leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// clear sticky faults
		leader.clearFaults();
		follower.clearFaults();

		// register sensors
		positionHist = SensorThread.getInstance().register(() -> Radians.of(encoder.getPosition()));
		timestamps = SensorThread.getInstance().makeTimestampQueue();
	}

	public void update() {
		this.data.leaderConnected = leader.getLastError() != REVLibError.kOk;
		this.data.leaderVoltage = Volts.of(leader.getBusVoltage() * leader.getAppliedOutput());
		this.data.leaderCurrent = Amps.of(leader.getOutputCurrent());

		this.data.followerConnected = follower.getLastError() != REVLibError.kOk;
		this.data.followerVoltage = Volts.of(follower.getBusVoltage() * follower.getAppliedOutput());
		this.data.followerCurrent = Amps.of(follower.getOutputCurrent());

		this.data.position = Radians.of(encoder.getPosition());
		this.data.velocity = RadiansPerSecond.of(encoder.getVelocity());

		this.data.timestamps = IterUtil.toDoubleArray(timestamps.stream());
		this.data.positions = IterUtil.toDoubleArray(positionHist.stream(), Radians);
	}

	public void setDesiredPosition(Angle position) {
		controller.setReference(position.in(Radians), ControlType.kPosition);
	}
}
