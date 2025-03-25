package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

public class PathChooser {
    SendableChooser<String> starting;
    SendableChooser<String> score;
    SendableChooser<String> intake;

    public PathChooser() {
        starting = new SendableChooser<>();
        score = new SendableChooser<>();
        intake = new SendableChooser<>();

        starting.addOption("Position 1", "1");
        starting.addOption("Position 2", "2");
        starting.addOption("Position 3", "3");

        starting.setDefaultOption("Disable", "disable");
        score.setDefaultOption("Disable", "disable");
        intake.setDefaultOption("Disable", "disable");

        SmartDashboard.putData("Starting Position", starting);
        SmartDashboard.putData("Score Path", score);
        SmartDashboard.putData("Intake Path", intake);

        starting.onChange(this::updateScoreOptions);
    }

    void updateScoreOptions(String starting) {
        score = new SendableChooser<>();
        score.addOption("Back Right", "brscore");
        score.addOption("Back Left", "blscore");
        score.addOption("Front Right", "frscore");
        score.setDefaultOption("Front Left", "flscore");
        score.addOption("Back", "bscore");

        SmartDashboard.putData("Score Path", score);
        score.onChange(this::updateIntakeOptions);

        this.updateIntakeOptions("flscore");
    }

    void updateIntakeOptions(String score) {
        intake = new SendableChooser<>();

        if (!score.substring(0, 2).contains("r")) {
            intake.addOption("Left", "lintake");
        }
        if (!score.contains("l")) {
            intake.addOption("Right", "rintake");
        }

        intake.setDefaultOption("Disable", "disable");
        SmartDashboard.putData("Intake Path", intake);
    }

    public List<String> trajectories() {
        if (starting.getSelected().equals("disable")) {
            return List.of();
        } else if (intake.getSelected().equals("disable")) {
            return List.of(starting.getSelected() + "_" + score.getSelected());
        } else {
            return List.of(
                starting.getSelected() + "_" + score.getSelected(),
                score.getSelected() + "_" + intake.getSelected()
            );
        }
    }
}
