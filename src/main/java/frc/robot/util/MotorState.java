package frc.robot.util;

public record MotorState(boolean connected, double position, double velocity, double voltage, double current) {
    public static MotorState defaultState() {
        return new MotorState(false, 0, 0, 0, 0);
    }
}
