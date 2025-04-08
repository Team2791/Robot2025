package frc.robot.subsystems.intake;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.util.Alerter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

public class IntakeSpark extends IntakeIO {
    final SparkMax leader;
    final SparkMax follower;

    final SparkLimitSwitch beam;

    public IntakeSpark() {
        leader = new SparkMax(IOConstants.Intake.kLeft, SparkMax.MotorType.kBrushless);
        follower = new SparkMax(IOConstants.Intake.kRight, SparkMax.MotorType.kBrushless);
        beam = leader.getReverseLimitSwitch();

        leader.clearFaults();
        follower.clearFaults();

        leader.configure(
            SparkConfigConstants.Intake.kLeft,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        follower.configure(
            SparkConfigConstants.Intake.kRight,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        Alerter.getInstance().registerSpark("IntakeLeader", leader);
        Alerter.getInstance().registerSpark("IntakeFollower", follower);
    }

    @Override
    public void update() {
        this.data.leftConnected = leader.getLastError() == REVLibError.kOk;
        this.data.leftVoltage = Volts.of(leader.getBusVoltage() * leader.getAppliedOutput());
        this.data.leftCurrent = Amps.of(leader.getOutputCurrent());
        this.data.leftPower = leader.getAppliedOutput();

        this.data.rightConnected = follower.getLastError() == REVLibError.kOk;
        this.data.rightVoltage = Volts.of(follower.getBusVoltage() * follower.getAppliedOutput());
        this.data.rightCurrent = Amps.of(follower.getOutputCurrent());
        this.data.rightPower = follower.getAppliedOutput();

        this.data.broken = beam.isPressed();
    }

    @Override
    public void set(double left, double right) {
        assert Math.abs(left) <= 1.0 : "Needed -1.0 <= left <= 1.0, got %f".formatted(left);
        assert Math.abs(right) <= 1.0 : "Needed -1.0 <= right <= 1.0, got %f".formatted(right);

        leader.set(left);
        follower.set(right);
    }
}
