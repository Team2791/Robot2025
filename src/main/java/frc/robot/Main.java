package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public final class Main {
    private Main() { }

    public static void main(String... args) {
        RobotBase.startRobot(() -> {
            try {
                return new Robot();
            } catch (IOException | ParseException e) {
                System.out.println(e);
                e.printStackTrace();
                System.exit(1); // panic
                return null;
            }
        });
    }
}
