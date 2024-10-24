package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class TeleopDriveCommand extends Command {
    Drivetrain drivetrain;
    CommandXboxController controller;
    Boolean fieldRelative;

    public TeleopDriveCommand(Drivetrain drivetrain, CommandXboxController controller, Boolean fieldRelative) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.fieldRelative = fieldRelative;

        addRequirements(drivetrain);
    }

    /**
     * Begin moving
     */
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (fieldRelative) {

        } else {
            drivetrain.setCurrentSpeeds(
                    new ChassisSpeeds(controller.getLeftX(), controller.getRightX(), controller.getRightX()));
        }

        // Consider the following
        // double rcw = pJoystick -> GetTwist();
        // double forwrd = pJoystick -> GetY() * -1; /* Invert stick Y axis */
        // double strafe = pJoystick -> GetX();

        // float pi = 3.1415926;

        // /* Adjust Joystick X/Y inputs by navX MXP yaw angle */

        // double gyro_degrees = ahrs -> GetYaw();
        // float gyro_radians = gyro_degrees * pi / 180;
        // float temp = forwrd * cos(gyro_radians) +
        // strafe * sin(gyro_radians);
        // strafe = -forwrd * sin(gyro_radians) +
        // strafe * cos(gyro_radians);
        // fwd = temp;

        // /* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
        /* rotated by the gyro angle, and can be sent to drive system */
        // just looked again and realised you might not have finsished with this since
        // sticks are idk, but thought i would leave comments

        // MICHAEL: 1, remeber joystick space is from -1 to 1, and chassis speeds is
        // meters and radians per second, so currenlty the speed of this robot is capped
        // at 1 meter per second

        // MICHAEL: 2, easy to get this mixed up by the coordinate system of joysticks
        // basically opposite to robot.
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        // For robot: https://docs.wpilib.org/en/stable/_images/robot-2d.svg
        // For joystick: https://docs.wpilib.org/en/stable/_images/joystick-3d.svg
        // (pretend each xbox stick is joystick, you can ignore Z twist)

        // Michael: 3, wtf are stick choices

        // Michael: 4, extra not required feature. we like to square and copy sign of
        // rotation speed from controller to make nicer driver exsperence (better
        // control at low speeds, quick ramp to fast speeds)

        drivetrain.setCurrentSpeeds(
                new ChassisSpeeds(controller.getLeftX(), controller.getRightX(), controller.getRightX()));
    }

    @Override
    public boolean isFinished() {
        // Michael: returning true here means isFinished causes the command to run for
        // only one execute cycle then stop
        return true;
    }

    /**
     * Turn off motors when we don't want the robot to move anymore
     */
    //
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}