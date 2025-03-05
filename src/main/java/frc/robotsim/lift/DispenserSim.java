package frc.robotsim.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.DispenserConstants.Motor;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robotio.lift.DispenserIO;

import static edu.wpi.first.units.Units.*;

public class DispenserSim extends DispenserIO {
    final SparkMax motor;
    final SparkMaxSim motorSim;
    final RelativeEncoder encoder;

    final FlywheelSim mechanism;

    public DispenserSim() {
        DCMotor gearbox = DCMotor.getNEO(2);

        motor = new SparkMax(IOConstants.Dispenser.kLeader, MotorType.kBrushless);
        encoder = motor.getEncoder();

        motor.configure(
            SparkConfigConstants.Dispenser.kLeader,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        motorSim = new SparkMaxSim(motor, gearbox);
        mechanism = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(gearbox, Motor.kMoI, Motor.kReduction),
            gearbox
        );
    }

    @Override
    public void update() {
        // update wpi mech
        mechanism.setInputVoltage(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        mechanism.update(0.02);

        // update spark sim
        motorSim.iterate(
            mechanism.getAngularVelocityRadPerSec(),
            RoboRioSim.getVInVoltage(),
            0.02
        );

        // account for current draw
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(mechanism.getCurrentDrawAmps()));

        // data
        this.data.leaderConnected = false;
        this.data.leaderVoltage = Volts.of(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        this.data.leaderCurrent = Amps.of(mechanism.getCurrentDrawAmps());

        this.data.followerConnected = false;
        this.data.followerVoltage = this.data.leaderVoltage.unaryMinus();
        this.data.followerCurrent = this.data.leaderCurrent;

        this.data.velocity = RadiansPerSecond.of(motor.get());
    }

    @Override
    public void set(double power) {
        motor.set(power);
    }
}
