package frc.robot.subsystems.dispenser;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.util.Alerter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

public class DispenserSpark extends DispenserIO {
    final SparkMax leader;
    final SparkMax follower;

    final SparkLimitSwitch beam;

    public DispenserSpark() {
        leader = new SparkMax(IOConstants.Dispenser.kLeader, MotorType.kBrushless);
        follower = new SparkMax(IOConstants.Dispenser.kFollower, MotorType.kBrushless);
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

        Alerter.getInstance().registerSpark("DispenserLeader", leader);
        Alerter.getInstance().registerSpark("DispenserFollower", follower);
    }

    @Override
    @SuppressWarnings("DuplicatedCode")
    public void update() {
        this.data.leaderConnected = leader.getLastError() == REVLibError.kOk;
        this.data.leaderVoltage = Volts.of(leader.getBusVoltage() * leader.getAppliedOutput());
        this.data.leaderCurrent = Amps.of(leader.getOutputCurrent());

        this.data.followerConnected = follower.getLastError() == REVLibError.kOk;
        this.data.followerVoltage = Volts.of(follower.getBusVoltage() * follower.getAppliedOutput());
        this.data.followerCurrent = Amps.of(follower.getOutputCurrent());

        this.data.power = leader.getAppliedOutput();
        this.data.broken = beam.isPressed();
    }

    @Override
    public void set(double power) {
        leader.set(power);
    }
}
