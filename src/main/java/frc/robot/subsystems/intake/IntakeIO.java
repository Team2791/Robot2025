package frc.robot.subsystems.intake;

import frc.robot.util.MotorState;
import org.littletonrobotics.junction.AutoLog;

public abstract class IntakeIO {
    public final IntakeDataAutoLogged data = new IntakeDataAutoLogged();

    public abstract void update();

    public abstract void intake(double power);

    public abstract void pivot(double power);


    @AutoLog
    public static class IntakeData {
        public MotorState intake = MotorState.defaultState();
        public MotorState pivot = MotorState.defaultState();
    }
}
