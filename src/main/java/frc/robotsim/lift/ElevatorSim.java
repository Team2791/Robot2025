package frc.robotsim.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robotio.lift.ElevatorIO;

import static edu.wpi.first.units.Units.*;

public class ElevatorSim extends ElevatorIO {
    final SparkFlex motor;
    final SparkFlexSim motorSim;

    final RelativeEncoder encoder;
    final SparkClosedLoopController controller;

    final edu.wpi.first.wpilibj.simulation.ElevatorSim elevatorSim;

    public ElevatorSim() {
        DCMotor gearbox = DCMotor.getNeoVortex(2);

        motor = new SparkFlex(IOConstants.Elevator.kLeader, MotorType.kBrushless);
        controller = motor.getClosedLoopController();

        motor.configure(
            SparkConfigConstants.Elevator.kLeader,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        motorSim = new SparkFlexSim(motor, gearbox);
        encoder = motor.getEncoder();

        elevatorSim = new edu.wpi.first.wpilibj.simulation.ElevatorSim(
            LinearSystemId.createElevatorSystem(
                gearbox,
                ElevatorConstants.Carriage.kMass,
                ElevatorConstants.Sprocket.kRadius,
                ElevatorConstants.Motor.kReduction * ElevatorConstants.Carriage.kFactor
            ),
            gearbox,
            0.0,
            ElevatorConstants.Heights.kL4,
            true,
            0.0
        );
    }

    @Override
    public void update() {
        // update the wpilib mech
        elevatorSim.setInput(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        elevatorSim.update(0.02);

        double carriageVelocity = elevatorSim.getVelocityMetersPerSecond();
        double elevatorVelocity = carriageVelocity / ElevatorConstants.Carriage.kFactor;
        double encoderVelocity = elevatorVelocity / ElevatorConstants.Sprocket.kRadius;
        double motorVelocity = encoderVelocity / ElevatorConstants.Motor.kReduction;

        // update the motor sim to make it move
        motorSim.iterate(
            motorVelocity,
            RoboRioSim.getVInVoltage(),
            0.02
        );

        // make sure we account for differences in battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

        this.data.leaderConnected = false;
        this.data.leaderVoltage = Volts.of(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        this.data.leaderCurrent = Amps.of(elevatorSim.getCurrentDrawAmps());

        this.data.followerConnected = false;
        this.data.followerVoltage = this.data.leaderVoltage.unaryMinus();
        this.data.followerCurrent = this.data.leaderCurrent;

        this.data.position = Radians.of(encoder.getPosition());
        this.data.velocity = RadiansPerSecond.of(encoder.getVelocity());
    }

    @Override
    public void setDesiredPosition(Angle position) {
        controller.setReference(position.in(Radians), ControlType.kPosition);
    }
}
