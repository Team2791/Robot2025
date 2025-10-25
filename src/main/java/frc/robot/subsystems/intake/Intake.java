package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private boolean hasBall = false;
    private IntakeState currentIntakeState = IntakeState.Hold;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void intake(IntakeState state) {
        this.io.intake(
                switch (state) {
                    case Intake -> ControlConstants.Intake.Power.kIntake;
                    case Outtake -> ControlConstants.Intake.Power.kOuttake;
                    case Hold -> hasBall ? ControlConstants.Intake.Power.kHold : 0;
                });

        this.currentIntakeState = state;
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

        Logger.processInputs("Intake", io.data);

        hasBall = switch (this.currentIntakeState) {
            case Intake -> Math.abs(io.data.intake.velocity()) < 1788;
            case Outtake -> false;
            case Hold -> Math.abs(io.data.intake.velocity()) < 314;
        };
    }

    public enum IntakeState {
        Intake,
        Outtake,
        Hold,
    }

    public enum PivotState {
        Up,
        Down,
        Stop,
    }
}
