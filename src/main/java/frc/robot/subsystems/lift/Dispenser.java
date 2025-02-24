package frc.robot.subsystems.lift;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.logging.Alerter;
import frc.robot.logging.threads.SensorThread;
import frc.robot.util.IterUtil;
import frc.robotio.scoring.DispenserIO;

import java.util.Queue;

import static edu.wpi.first.units.Units.*;

public class Dispenser extends DispenserIO {
    final SparkMax leader;
    final SparkMax follower;

    final SparkLimitSwitch beam;
    final RelativeEncoder encoder;

    final Queue<Double> timestamps;
    final Queue<AngularVelocity> velocityHist;
    final Queue<Boolean> brokenHist;

    public Dispenser() {
        leader = new SparkMax(IOConstants.Dispenser.kLeader, MotorType.kBrushless);
        follower = new SparkMax(IOConstants.Dispenser.kFollower, MotorType.kBrushless);

        encoder = leader.getEncoder();
        beam = leader.getReverseLimitSwitch();

        leader.clearFaults();
        follower.clearFaults();

        leader.configure(
            SparkConfigConstants.Dispenser.kLeader,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        follower.configure(
            SparkConfigConstants.Dispenser.kFollower,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        timestamps = SensorThread.getInstance().makeTimestampQueue();
        velocityHist = SensorThread.getInstance().register(() -> RadiansPerSecond.of(encoder.getVelocity()));
        brokenHist = SensorThread.getInstance().register(beam::isPressed);

        Alerter.getInstance().registerSpark("DispenserLeader", leader);
        Alerter.getInstance().registerSpark("DispenserFollower", follower);
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
        this.data.broken = beam.isPressed();

        this.data.timestamps = IterUtil.toDoubleArray(timestamps.stream());
        this.data.velocities = IterUtil.toDoubleArray(velocityHist.stream(), RadiansPerSecond);
        this.data.brokens = IterUtil.toBoolArray(brokenHist);
    }

    @Override
    public void set(double power) {
        leader.set(power);
    }
}
