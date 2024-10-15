package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class TeleopDriveCommand extends Command {
    Drivetrain drivetrain;
    CommandXboxController controller;

    public TeleopDriveCommand(Drivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
    }

    /**
     * Begin moving
     */
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drivetrain.setSpeeds(new ChassisSpeeds(controller.getLeftX(), controller.getRightX(), controller.getRightX()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * Turn off motors when we don't want the robot to move anymore
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}