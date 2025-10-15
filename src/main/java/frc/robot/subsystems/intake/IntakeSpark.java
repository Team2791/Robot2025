package frc.robot.subsystems.intake;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.IOConstants;
import frc.robot.util.MotorState;

public class IntakeSpark extends IntakeIO {
    final SparkMax motor;

    public IntakeSpark() {
        this.motor = new SparkMax(IOConstants.Intake.kId, MotorType.kBrushless);
    }

    @Override
    public void set(double power) {
        motor.set(power);
    }

    @Override
    public void update() {
        this.data.intake = new MotorState(
                this.motor.getLastError() == REVLibError.kOk,
                this.motor.getEncoder().getPosition(),
                this.motor.getEncoder().getVelocity(),
                this.motor.getAppliedOutput() * this.motor.getBusVoltage(),
                this.motor.getOutputCurrent());
    }
}
