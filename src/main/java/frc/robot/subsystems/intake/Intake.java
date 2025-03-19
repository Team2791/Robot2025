package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AdvantageUtil;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    final IntakeIO intake;

    public Intake(IntakeIO intake) {
        this.intake = intake;
    }

    public IntakeIO.IntakeData data() { return intake.data; }

    public void set(double left, double right) {
        intake.set(left, right);
    }

    public void set(double power) {
        set(power, -power);
    }

    public void stop() {
        set(0, 0);
    }

    @Override
    public void periodic() {
        intake.update();

        Logger.processInputs("Intake", intake.data);
        AdvantageUtil.logActiveCommand(this);
    }
}
