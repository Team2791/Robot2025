package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AdvantageUtil;
import org.littletonrobotics.junction.Logger;

public class AlgaeManipulator extends SubsystemBase {
    final ManipulatorIO manipulator;

    public AlgaeManipulator(ManipulatorIO manipulator) {
        this.manipulator = manipulator;
        this.toggle(false);
    }

    public void toggle(boolean enabled) {
        manipulator.toggle(enabled);
    }

    public ManipulatorIO.ManipulatorData data() {
        return manipulator.data;
    }

    @Override
    public void periodic() {
        manipulator.update();

        Logger.processInputs("AlgaeManipulator", manipulator.data);
        AdvantageUtil.logActiveCommand(this);
    }
}
