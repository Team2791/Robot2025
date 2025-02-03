package frc.robot;


import java.io.IOException;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
	private Main() {}

	public static void main(String... args) {
		RobotBase.startRobot(() -> {
			try {
				return new Robot();
			} catch (IOException | ParseException e) {
				e.printStackTrace();
				System.exit(1); // panic
				return null;
			}
		});
	}
}
