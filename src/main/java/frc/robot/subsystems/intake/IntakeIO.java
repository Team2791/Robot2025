package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public abstract class IntakeIO {
    @AutoLog
    public static class IntakeData {
        public boolean leftConnected = false;
        public AngularVelocity leftVelocity = RadiansPerSecond.of(0);
        public Voltage leftVoltage = Volts.of(0);
        public Current leftCurrent = Amps.of(0);

        public boolean rightConnected = false;
        public AngularVelocity rightVelocity = RadiansPerSecond.of(0);
        public Voltage rightVoltage = Volts.of(0);
        public Current rightCurrent = Amps.of(0);

        public boolean broken = false;
    }

    public final IntakeDataAutoLogged data = new IntakeDataAutoLogged();

    public abstract void update();

    public abstract void set(double left, double right);
}
