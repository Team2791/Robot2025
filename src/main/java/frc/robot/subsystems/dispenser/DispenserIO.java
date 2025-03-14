package frc.robot.subsystems.dispenser;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

public abstract class DispenserIO {
    @AutoLog
    public static class DispenserData {
        public boolean leaderConnected = false;
        public Voltage leaderVoltage = Volts.of(0);
        public Current leaderCurrent = Amps.of(0);

        public boolean followerConnected = false;
        public Voltage followerVoltage = Volts.of(0);
        public Current followerCurrent = Amps.of(0);

        public double power = 0;
        public boolean broken = false;
    }

    public final DispenserDataAutoLogged data = new DispenserDataAutoLogged();

    public abstract void update();

    public abstract void set(double power);
}
