package frc.robot.subsystems.intake;

public class Intake {
    private final IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    // TODO: Add methods to control the intake via constants
    public void set(double power) {
        io.set(power);
    }

    public IntakeIO.IntakeData getData() {
        return io.data;
    }

    public void periodic() {
        io.update();
    }
}
