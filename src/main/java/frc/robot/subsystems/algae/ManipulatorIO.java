package frc.robot.subsystems.algae;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public abstract class ManipulatorIO {
    @AutoLog
    public static class ManipulatorData {
        public boolean turnConnected = false;
        public Voltage turnVoltage = Volts.of(0);
        public Current turnCurrent = Amps.of(0);
        public Angle turnPosition = Radians.of(0);

        public boolean spinConnected = false;
        public Voltage spinVoltage = Volts.of(0);
        public Current spinCurrent = Amps.of(0);
        public AngularVelocity spinVelocity = RadiansPerSecond.of(0);
    }

    public final ManipulatorData data = new ManipulatorData();
}
