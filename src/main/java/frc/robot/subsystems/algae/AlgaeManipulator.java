package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeManipulator extends SubsystemBase {
    final ManipulatorIO manipulator;

    public AlgaeManipulator(ManipulatorIO manipulator) {
        this.manipulator = manipulator;
    }

    public void toggle(boolean enabled) {
        manipulator.toggle(enabled);
    }

    public ManipulatorIO.ManipulatorData data() {
        return manipulator.data;
    }
}
