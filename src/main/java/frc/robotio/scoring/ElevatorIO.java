package frc.robotio.scoring;

import edu.wpi.first.units.measure.*;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public abstract class ElevatorIO {
    @AutoLog
    public static class ElevatorData {
        public boolean leaderConnected = false;
        public Voltage leaderVoltage = Volts.of(0);
        public Current leaderCurrent = Amps.of(0);

        public boolean followerConnected;
        public Voltage followerVoltage = Volts.of(0);
        public Current followerCurrent = Amps.of(0);

        public Angle position;
        public AngularVelocity velocity;

        public double[] timestamps = new double[0];
        public double[] positions = new double[0];
    }

    public final ElevatorDataAutoLogged data = new ElevatorDataAutoLogged();

    public abstract void update();

    protected abstract void setDesiredPosition(Angle position);

    public final void setDesiredPosition(Distance position) {
        double height = position.in(Meters);
        double angle = height / ElevatorConstants.Drum.kRadius;
        setDesiredPosition(Radians.of(angle));
    }
}
