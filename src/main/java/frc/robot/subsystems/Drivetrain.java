package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class Drivetrain extends SubsystemBase {
        private SwerveModule frontLeftSwerveModule;
        private SwerveModule frontRightSwerveModule;
        private SwerveModule backLeftSwerveModule;
        private SwerveModule backRightSwerveModule;

        private SwerveDriveKinematics kinematics;

        // MICHAEL: Not something you should have known, but i suggest using a SwerveDrivePoseEstimator,
        // The pose estimators allow for easy adding of vision measurements with their Kalman filter.
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
        private SwerveDriveOdometry odometry;

        private AHRS gyro; // MICHAEL: Since when you started this eletrical put a Pigion2 on the bot instead of a navx gyro. Pigion2 is from CTRE who makes our CANCoders so you should have the libary
        // not something you should have done but just letting you know. 

        private Pose2d targetPose = new Pose2d(); // MICHAEL: not sure if this is still needed.

        private Pose2d currentPose = new Pose2d();

        private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);

        private ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5); // MICHAEL: before testing make sure to set this to zero

        //MICHAEL: I personally like dependency injection for this kind of thing
        public Drivetrain() {
                Translation2d frontLeftSwervePosition = new Translation2d(SwerveModuleConstants.MODULE_LOCATION_X,
                                SwerveModuleConstants.MODULE_LOCATION_Y);
                Translation2d frontRightSwervePosition = new Translation2d(SwerveModuleConstants.MODULE_LOCATION_X,
                                -SwerveModuleConstants.MODULE_LOCATION_Y);
                Translation2d backLeftSwervePosition = new Translation2d(-SwerveModuleConstants.MODULE_LOCATION_X,
                                SwerveModuleConstants.MODULE_LOCATION_Y);
                Translation2d backRightSwervePosition = new Translation2d(-SwerveModuleConstants.MODULE_LOCATION_X,
                                -SwerveModuleConstants.MODULE_LOCATION_Y);

                frontLeftSwerveModule = new SwerveModule(SwerveModuleConstants.VELOCITY_MOTOR_ID_FL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ID_FL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FL,
                                frontLeftSwervePosition);

                frontRightSwerveModule = new SwerveModule(
                                SwerveModuleConstants.VELOCITY_MOTOR_ID_FR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ID_FR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FR,
                                frontRightSwervePosition);

                backLeftSwerveModule = new SwerveModule(
                                SwerveModuleConstants.VELOCITY_MOTOR_ID_BL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ID_BL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BL,
                                backLeftSwervePosition);

                backRightSwerveModule = new SwerveModule(
                                SwerveModuleConstants.VELOCITY_MOTOR_ID_BR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ID_BR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BR,
                                backRightSwervePosition);

                kinematics = new SwerveDriveKinematics(
                                frontLeftSwervePosition,
                                frontRightSwervePosition,
                                backLeftSwervePosition,
                                backRightSwervePosition);

                odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getAngle()),
                                new SwerveModulePosition[] {
                                                frontLeftSwerveModule.getPosition(),
                                                frontRightSwerveModule.getPosition(),
                                                backLeftSwerveModule.getPosition(),
                                                backRightSwerveModule.getPosition()
                                });
        }

        // MICHAEL: could use SwerveDriveWheelPositions here to wrap array
        public SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] {
                                frontLeftSwerveModule.getPosition(),
                                frontRightSwerveModule.getPosition(),
                                backLeftSwerveModule.getPosition(),
                                backRightSwerveModule.getPosition()
                };
        }

        // MICHASLFJLL: I would maybe add a get SwerveDriveWheelStates method here, rarly used but helpful if you are gonna put it on AdvantageScope

        // MICHAEL: Also helpful to add a getSpeeds method. Kinematics can do that for you. This is needed Holonomic path following (which is used by pathplanner etc)

        public void setSpeeds(ChassisSpeeds speeds) {
                // MIKE: Currenly your speeds are for continuous-time, chassis speeds can make them into speeds that are meant 
                // to be used at a single time step, so each component (x, y omega) can run correclty (discretization).
                // don't worry about math, chassis speeds has method for discretize
                // Basicly discretize converts speeds that are meant to be followed for a while into speeds that are only exspected to be followed for 20ms until new speeds are set
                targetChassisSpeeds = speeds;

                // MICHALE: IN MY opinion (just preference), it like to break up setSpeeds(ChassisSpeeds) setWheelSpeeds(SwerveDriveWheelStates) into two different methods incase you ever want to just set wheel speeds for debugging
                // ALSO IN MY opinion, setting wheel speeds should be a set and forget thing, where you do it in you method and don't have to recompute wheel states every perodic cycle. again just preference
        }

        public void stop() {
                targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);
        }

        @Override
        public void periodic() {
                currentPose = odometry.update(gyro.getRotation2d(), getModulePositions());

                SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetChassisSpeeds);

                // Michael: currently wheel speeds could exceed the max possible velocity, which is ok by itself as it will just mean the robot fails to reach those speeds.
                // but it may cause a bad drift in driven angle from desired angle. ie if x is trying to go 150% and y is going 100%, but max is 100 percent
                // so it goes diagnal when you mean to go more x forward. you probable had to deal with this in FTC
                // kinematics has a desature method just for this.

                frontLeftSwerveModule.setTargetState(states[0]);
                frontRightSwerveModule.setTargetState(states[1]);
                backLeftSwerveModule.setTargetState(states[2]);
                backRightSwerveModule.setTargetState(states[3]);
        }
}
