package frc.robot.commands.dispenser;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DispenserConstants;
import frc.robot.subsystems.dispenser.Dispenser;

public class SlowBack extends Command {
    final Dispenser dispenser;
    final Timer timer = new Timer(); // todo: replace with encoder readings
    boolean wasBroken = false;

    /**
     * Run the coral back through the dispenser slowly
     *
     * @param dispenser the dispenser subsystem
     */
    public SlowBack(Dispenser dispenser) {
        this.dispenser = dispenser;
        addRequirements(dispenser);
    }

    @Override
    public void initialize() {
        wasBroken = false;
        dispenser.dispense(DispenserConstants.Power.kSlowBack);
    }

    @Override
    public void execute() {
        if (dispenser.data().broken && !wasBroken) {
            timer.restart();
            wasBroken = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        dispenser.dispense(0);
    }

    @Override
    public boolean isFinished() {
        return wasBroken && timer.get() >= 0.1;
    }
}
