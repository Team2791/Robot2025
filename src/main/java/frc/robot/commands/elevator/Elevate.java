package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.ElevatorConstants;
import frc.robot.event.Emitter;
import frc.robot.event.Key;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.elevator.Elevator;

public class Elevate extends FunctionWrapper {
    private static Key<Double, Dispenser.ReefRange> retract;

    /**
     * Elevate to a certain height. This command will block until the lift is at the desired height.
     *
     * @param lift   The lift subsystem
     * @param height The height to elevate to
     */
    public Elevate(Elevator lift, int height) {
        this(lift, height, true);
    }

    /**
     * Elevate to a certain height. Decides whether to block or not.
     *
     * @param elevator The elevator subsystem
     * @param height   The height to elevate to
     * @param blocking Whether to block until the elevator is at the desired height
     */
    public Elevate(Elevator elevator, int height, boolean blocking) {
        super(
            () -> elevator.elevate(height),
            () -> elevator.atLevel(height) || !blocking,
            elevator
        );
    }

    public static void registerRetract(Elevator elevator) {
        final Elevate instance = new Elevate(elevator, 0, true);
        final CommandScheduler scheduler = CommandScheduler.getInstance();

        retract = Emitter.on(
            new Dispenser.ReefRange(),
            distance -> {
                boolean outside = distance >= ElevatorConstants.Range.kRetract;
                boolean scheduled = scheduler.isScheduled(instance);
                boolean zeroed = elevator.atLevel(0);
                boolean auto = DriverStation.isAutonomous();

                if (auto) return;
                if (outside && !scheduled && !zeroed) scheduler.schedule(instance);
            }
        );
    }

    public static void disableRetract() {
        if (retract == null) return;
        Emitter.off(retract);
    }
}
