package frc.robot.subsystems.dispenser;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DispenserConstants;
import frc.robot.subsystems.dispenser.DispenserIO.DispenserData;
import frc.robot.util.AdvantageUtil;
import org.littletonrobotics.junction.Logger;

public class Dispenser extends SubsystemBase {
    final DispenserIO dispenser;

    public Dispenser(DispenserIO dispenser) {
        this.dispenser = dispenser;
    }

    /** Run the dispenser */
    public void dispense() {
        dispense(DispenserConstants.Power.kDispense);
    }

    /** Run the dispenser (with power) */
    public void dispense(double power) {
        assert Math.abs(power) <= 1.0 : "Needed -1.0 <= power <= 1.0, got %f".formatted(power);
        dispenser.set(power);
    }

    /** Get a copy of the dispenser data */
    public DispenserData data() {
        return dispenser.data.clone();
    }

    @Override
    public void periodic() {
        dispenser.update();

        Logger.processInputs("Dispenser", dispenser.data);
        AdvantageUtil.logActiveCommand(this);
    }
}
