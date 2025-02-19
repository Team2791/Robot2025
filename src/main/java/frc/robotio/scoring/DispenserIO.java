package frc.robotio.scoring;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public abstract class DispenserIO {
    @AutoLog
    public static class DispenserData {
    }

    public final DispenserDataAutoLogged data = new DispenserDataAutoLogged();

    public abstract void update();
}
