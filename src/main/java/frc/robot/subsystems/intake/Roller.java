package frc.robot.subsystems.intake;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.logging.Alerter;
import frc.robotio.RollerIO;

import static edu.wpi.first.units.Units.*;

public class Roller extends RollerIO {
    final SparkMax leftMotor;
    final SparkMax rightMotor;

    final RelativeEncoder leftEncoder;
    final RelativeEncoder rightEncoder;

    final SparkLimitSwitch beam;

    public Roller() {
        leftMotor = new SparkMax(IOConstants.Roller.kLeft, SparkMax.MotorType.kBrushless);
        rightMotor = new SparkMax(IOConstants.Roller.kRight, SparkMax.MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        beam = leftMotor.getReverseLimitSwitch();

        leftMotor.clearFaults();
        rightMotor.clearFaults();

        leftMotor.configure(
            SparkConfigConstants.Roller.kLeft,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        rightMotor.configure(
            SparkConfigConstants.Roller.kRight,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        Alerter.getInstance().registerSpark("RollerLeft", leftMotor);
        Alerter.getInstance().registerSpark("RollerRight", rightMotor);
    }

    @Override
    public void update() {
        this.data.leftConnected = leftMotor.getLastError() != REVLibError.kOk;
        this.data.leftVelocity = RadiansPerSecond.of(leftMotor.getEncoder().getVelocity());
        this.data.leftVoltage = Volts.of(leftMotor.getBusVoltage() * leftMotor.getAppliedOutput());
        this.data.leftCurrent = Amps.of(leftMotor.getOutputCurrent());

        this.data.rightConnected = rightMotor.getLastError() != REVLibError.kOk;
        this.data.rightVelocity = RadiansPerSecond.of(rightMotor.getEncoder().getVelocity());
        this.data.rightVoltage = Volts.of(rightMotor.getBusVoltage() * rightMotor.getAppliedOutput());
        this.data.rightCurrent = Amps.of(rightMotor.getOutputCurrent());

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
