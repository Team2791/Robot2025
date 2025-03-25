package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

public class PathChooser {
    SendableChooser<String> startingChooser;
    SendableChooser<String> scoreChooser;
    SendableChooser<String> intakeChooser;

    public PathChooser() {
        startingChooser = new SendableChooser<>();
        scoreChooser = new SendableChooser<>();
        intakeChooser = new SendableChooser<>();

        startingChooser.addOption("Position 1", "1");
        startingChooser.addOption("Position 2", "2");
        startingChooser.addOption("Position 3", "3");

        startingChooser.setDefaultOption("Disable", "disable");
        scoreChooser.setDefaultOption("Disable", "disable");
        intakeChooser.setDefaultOption("Disable", "disable");

        SmartDashboard.putData("Starting Position", startingChooser);
        SmartDashboard.putData("Score Path", scoreChooser);
        SmartDashboard.putData("Intake Path", intakeChooser);

        startingChooser.onChange(this::updateScoreOptions);
    }

    void updateScoreOptions(String starting) {
        if (starting == null || starting.equals("disable")) {
            scoreChooser = new SendableChooser<>();
            scoreChooser.setDefaultOption("Disable", "disable");
            SmartDashboard.putData("Score Path", scoreChooser);
            this.updateIntakeOptions("disable");

            return;
        }

        scoreChooser = new SendableChooser<>();
        scoreChooser.addOption("Back Right", "brscore");
        scoreChooser.addOption("Back Left", "blscore");
        scoreChooser.addOption("Front Right", "frscore");
        scoreChooser.setDefaultOption("Front Left", "flscore");
        scoreChooser.addOption("Back", "bscore");

        SmartDashboard.putData("Score Path", scoreChooser);
        scoreChooser.onChange(this::updateIntakeOptions);

        this.updateIntakeOptions("flscore");
    }

    void updateIntakeOptions(String score) {
        if (score == null || score.equals("disable")) {
            intakeChooser = new SendableChooser<>();
            intakeChooser.setDefaultOption("Disable", "disable");
            SmartDashboard.putData("Intake Path", intakeChooser);

            return;
        }

        intakeChooser = new SendableChooser<>();

        if (!score.substring(0, 2).contains("r")) {
            intakeChooser.addOption("Left", "lintake");
        }

        if (!score.contains("l")) {
            intakeChooser.addOption("Right", "rintake");
        }

        intakeChooser.setDefaultOption("Disable", "disable");
        SmartDashboard.putData("Intake Path", intakeChooser);
    }

    public List<String> trajectories() {
        if (startingChooser.getSelected().equals("disable")) {
            return List.of();
        } else if (intakeChooser.getSelected().equals("disable")) {
            return List.of(startingChooser.getSelected() + "_" + scoreChooser.getSelected());
        } else {
            return List.of(
                startingChooser.getSelected() + "_" + scoreChooser.getSelected(),
                scoreChooser.getSelected() + "_" + intakeChooser.getSelected()
            );
        }
    }
}
