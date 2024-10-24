package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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

        // MICHAEL: Not something you should have known, but i suggest using a
        // SwerveDrivePoseEstimator,
        // The pose estimators allow for easy adding of vision measurements with their
        // Kalman filter.
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
        private SwerveDriveOdometry odometry;

        private Pigeon2 gyro;

        private ChassisSpeeds targetSpeeds = new ChassisSpeeds(0, 0, 0);
        private ChassisSpeeds currentSpeeds = new ChassisSpeeds(0, 0, 0);

        public Drivetrain(SwerveModule frontLeftSwerveModule, SwerveModule frontRightSwerveModule,
                        SwerveModule backLeftSwerveModule, SwerveModule backRightSwerveModule) {
                kinematics = new SwerveDriveKinematics(
                                frontLeftSwerveModule.getOffset(),
                                frontRightSwerveModule.getOffset(),
                                backLeftSwerveModule.getOffset(),
                                backRightSwerveModule.getOffset());

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

        // MICHASLFJLL: I would maybe add a get SwerveDriveWheelStates method here,
        // rarly used but helpful if you are gonna put it on AdvantageScope

        // MICHAEL: Also helpful to add a getSpeeds method. Kinematics can do that for
        // you. This is needed Holonomic path following (which is used by pathplanner
        // etc)

        public void setCurrentSpeeds(ChassisSpeeds speeds) {
                // MIKE: Currenly your speeds are for continuous-time, chassis speeds can make
                // them into speeds that are meant
                // to be used at a single time step, so each component (x, y omega) can run
                // correclty (discretization).
                // don't worry about math, chassis speeds has method for discretize
                // Basicly discretize converts speeds that are meant to be followed for a while
                // into speeds that are only exspected to be followed for 20ms until new speeds
                // are set
                targetSpeeds = speeds;

                // MICHALE: IN MY opinion (just preference), it like to break up
                // setSpeeds(ChassisSpeeds) setWheelSpeeds(SwerveDriveWheelStates) into two
                // different methods incase you ever want to just set wheel speeds for debugging
                // ALSO IN MY opinion, setting wheel speeds should be a set and forget thing,
                // where you do it in you method and don't have to recompute wheel states every
                // perodic cycle. again just preference
        }

        public void stop() {
                targetSpeeds = new ChassisSpeeds(0, 0, 0);
        }

        @Override
        public void periodic() {
                currentPose = odometry.update(gyro.getRotation2d(), getModulePositions());

                SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);

                // Michael: currently wheel speeds could exceed the max possible velocity, which
                // is ok by itself as it will just mean the robot fails to reach those speeds.
                // but it may cause a bad drift in driven angle from desired angle. ie if x is
                // trying to go 150% and y is going 100%, but max is 100 percent
                // so it goes diagnal when you mean to go more x forward. you probable had to
                // deal with this in FTC
                // kinematics has a desature method just for this.

                frontLeftSwerveModule.setTargetState(states[0]);
                frontRightSwerveModule.setTargetState(states[1]);
                backLeftSwerveModule.setTargetState(states[2]);
                backRightSwerveModule.setTargetState(states[3]);
        }
}
