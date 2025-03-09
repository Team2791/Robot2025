package frc.robot.subsystems.elevator;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.util.Alerter;

import static edu.wpi.first.units.Units.*;

public class ElevatorSpark extends ElevatorIO {
	final SparkFlex follower;
	final SparkFlex leader;

	final RelativeEncoder encoder;
	final SparkClosedLoopController controller;

	public ElevatorSpark() {
		leader = new SparkFlex(IOConstants.Elevator.kLeader, SparkLowLevel.MotorType.kBrushless);
		follower = new SparkFlex(IOConstants.Elevator.kFollower, SparkLowLevel.MotorType.kBrushless);

		// apply configs
		leader.configure(
			SparkConfigConstants.Elevator.kLeader,
			SparkConfigConstants.kResetMode,
			SparkConfigConstants.kPersistMode
		);
		follower.configure(
			SparkConfigConstants.Elevator.kFollower,
			SparkConfigConstants.kResetMode,
			SparkConfigConstants.kPersistMode
		);

		encoder = leader.getEncoder();
		controller = leader.getClosedLoopController();

		// clear sticky faults
		leader.clearFaults();
		follower.clearFaults();

		// register sparks
		Alerter.getInstance().registerSpark("ElevatorLeader", leader);
		Alerter.getInstance().registerSpark("ElevatorFollower", follower);
	}

	@Override
	@SuppressWarnings("DuplicatedCode")
	public void update() {
		this.data.leaderConnected = leader.getLastError() != REVLibError.kOk;
		this.data.leaderVoltage = Volts.of(leader.getBusVoltage() * leader.getAppliedOutput());
		this.data.leaderCurrent = Amps.of(leader.getOutputCurrent());

		this.data.followerConnected = follower.getLastError() != REVLibError.kOk;
		this.data.followerVoltage = Volts.of(follower.getBusVoltage() * follower.getAppliedOutput());
		this.data.followerCurrent = Amps.of(follower.getOutputCurrent());

		this.data.velocity = RadiansPerSecond.of(encoder.getVelocity());
		this.data.position = Radians.of(encoder.getPosition());
	}

	@Override
	public void setDesiredPosition(Angle position) {
		controller.setReference(position.in(Radians), SparkBase.ControlType.kPosition);
	}
}
