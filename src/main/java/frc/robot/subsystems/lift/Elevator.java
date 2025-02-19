package frc.robot.subsystems.lift;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.thread.SensorThread;
import frc.robot.util.IterUtil;
import frc.robotio.scoring.ElevatorIO;

import java.util.Queue;

import static edu.wpi.first.units.Units.*;

public class Elevator extends ElevatorIO {
    protected final SparkFlex leader;
    protected final SparkFlex follower;

    final RelativeEncoder encoder;
    final SparkClosedLoopController controller;

    final Queue<Angle> positionHist;
    final Queue<Double> timestamps;

    public Elevator() {
        leader = new SparkFlex(IOConstants.Elevator.kLeader, MotorType.kBrushless);
        follower = new SparkFlex(IOConstants.Elevator.kFollower, MotorType.kBrushless);

        encoder = leader.getEncoder();
        controller = leader.getClosedLoopController();

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
