package frc.robot.subsystems.intake;

import frc.robot.constants.ControlConstants;

public class Intake {
    private final IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void intake(IntakeState state) {
        this.io.intake(
                switch (state) {
                    case Intake -> ControlConstants.Intake.Power.kIntake;
                    case Outtake -> ControlConstants.Intake.Power.kOuttake;
                    case Stop -> 0.0;
                });
    }

    public void pivot(PivotState state) {
        this.io.pivot(
                switch (state) {
                    case Up -> ControlConstants.Intake.Power.kPivot;
                    case Down -> -ControlConstants.Intake.Power.kPivot;
                    case Stop -> 0.0;
                });
    }

    public IntakeIO.IntakeData getData() {
        return io.data;
    }

    public void periodic() {
        io.update();
    }

    public enum IntakeState {
        Intake,
        Outtake,
        Stop,
    }

    public enum PivotState {
        Up,
        Down,
        Stop,
    }
}
