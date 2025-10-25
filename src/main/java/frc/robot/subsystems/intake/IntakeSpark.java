package frc.robot.subsystems.intake;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.IOConstants;
import frc.robot.util.MotorState;

public class IntakeSpark extends IntakeIO {
    final SparkMax intake;
    final SparkMax pivot;

    public IntakeSpark() {
        this.intake = new SparkMax(IOConstants.Intake.kIntake, MotorType.kBrushless);
        this.pivot = new SparkMax(IOConstants.Intake.kPivot, MotorType.kBrushless);
    }

    @Override
    public void intake(double power) {
        intake.set(power);
    }

    @Override
    public void pivot(double power) {
        pivot.set(power);
    }

    @Override
    public void update() {
        this.data.intake = new MotorState(
                this.intake.getLastError() == REVLibError.kOk,
                this.intake.getEncoder().getPosition(),
                this.intake.getEncoder().getVelocity(),
                this.intake.getAppliedOutput() * this.intake.getBusVoltage(),
                this.intake.getOutputCurrent());
    }
}
