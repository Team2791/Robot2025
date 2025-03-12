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
    final SparkMax leftMotor;
    final SparkMax rightMotor;

    final SparkLimitSwitch beam;

    public IntakeSpark() {
        leftMotor = new SparkMax(IOConstants.Intake.kLeft, SparkMax.MotorType.kBrushless);
        rightMotor = new SparkMax(IOConstants.Intake.kRight, SparkMax.MotorType.kBrushless);
        beam = leftMotor.getReverseLimitSwitch();

        leftMotor.clearFaults();
        rightMotor.clearFaults();

        leftMotor.configure(
            SparkConfigConstants.Intake.kLeft,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        rightMotor.configure(
            SparkConfigConstants.Intake.kRight,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        Alerter.getInstance().registerSpark("IntakeLeader", leftMotor);
        Alerter.getInstance().registerSpark("IntakeFollower", rightMotor);
    }

    @Override
    public void update() {
        this.data.leftConnected = leftMotor.getLastError() == REVLibError.kOk;
        this.data.leftVoltage = Volts.of(leftMotor.getBusVoltage() * leftMotor.getAppliedOutput());
        this.data.leftCurrent = Amps.of(leftMotor.getOutputCurrent());
        this.data.leftPower = leftMotor.getAppliedOutput();

        this.data.rightConnected = rightMotor.getLastError() == REVLibError.kOk;
        this.data.rightVoltage = Volts.of(rightMotor.getBusVoltage() * rightMotor.getAppliedOutput());
        this.data.rightCurrent = Amps.of(rightMotor.getOutputCurrent());
        this.data.rightPower = rightMotor.getAppliedOutput();

        this.data.broken = beam.isPressed();
    }

    @Override
    public void set(double left, double right) {
        assert Math.abs(left) <= 1.0 : "Needed -1.0 <= left <= 1.0, got left=%f".formatted(left);
        assert Math.abs(right) <= 1.0 : "Needed -1.0 <= right <= 1.0, got right=%f".formatted(right);

        leftMotor.set(left);
        rightMotor.set(right);
    }
}
