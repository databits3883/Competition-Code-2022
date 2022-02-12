// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class ClimbConstants{
        public static final int LENGTH_WINCH_CHANNEL = 13;
        public static final int LENGTH_WINCH_FOLLOWER_CHANNEL = 14;


        public static final int ANGLE_WINCH_CHANNEL = 1;
        public static final int SPRING_HOOK_SWITCH_CHANNEL = 0;

        public static final double LENGTH_WINCH_GEARING = 1;
        public static final double LENGTH_WINCH_CIRCUMFRENCE = 1;

        public static final int ANGLE_WINCH_PD_SLOT = 0;
        public static final double WINCH_CURRENT_THRESHOLD = 1; //change later

        public static final double CLIMB_ARM_EXTEND_SPEED = 0.5;

        public static final double HOOK_RELEASE_DISTANCE = 0.1;
        public static final double FULL_EXTENSION_DISTANCE = 1;

        public static final double ARM_DROP_SPEED=0.3;
        public static final double ARM_DROP_TIME=0.5;


        public static final double WINCH_DESLACK_SPEED = 0.1;
        public static final double ARM_PULL_FAST_SPEED=0.7;
        public static final double ARM_PULL_SLOW_SPEED = 0.4;
    }

    public final class LauncherContants{
        public static final int LEADER_CHANNEL = 20;
        public static final int FOLLOWER_CHANNEL = 21;
        public static final double ENCODER_POSITIONAL_CONVERSION = 1.0/3.0;
    }

    public static class IntakeConstants{
        public static final int INTAKE_CHANNEL = 16;
        public static final int RAISE_LOWER_CHANNEL = 17;
        public static final int EXTEND_LIMIT_CHANNEL = 0;
        public static final int RETRACT_LIMIT_CHANNEL=1;

        public static final double spinSpeed = -1;

        public static final double extendSpeed = 0.3;
        public static final double retractSpeed = -0.3;

        public static final double EXTENSION_TIMEOUT=3;
        public static final double RETRACT_TIMEOUT=3;

    }

    public static class StagingConstants{
        public static final int motorChannel = 0;
        public static final double motorSpeed = 0.5;
    }

    public static class DriveConstants{
        public static final double ROTATION_GEARING = 1.0;
        public static final double VELOCITY_GEARING = 1.0/6.86;
        public static final double WHEEL_CIRCUMFRENCE = Math.PI * 4*2.54 *0.01;

        public static final double DRIVE_TRACK_WIDTH = 0.43;
        public static final double DRIVE_TRACK_LENGTH = 0.73;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVE_TRACK_LENGTH/2, DRIVE_TRACK_WIDTH/2), //fron right 
            new Translation2d(-DRIVE_TRACK_LENGTH/2, DRIVE_TRACK_WIDTH/2), //rear right
            new Translation2d(-DRIVE_TRACK_LENGTH/2, -DRIVE_TRACK_WIDTH/2), //rear left
            new Translation2d(DRIVE_TRACK_LENGTH/2, -DRIVE_TRACK_WIDTH/2) //front left
          );
        
        
        public static final double MAX_WHEEL_SPEED = 4.28;

        public static final double MAX_TURN_SPEED = MAX_WHEEL_SPEED * (Math.sqrt(DRIVE_TRACK_LENGTH*DRIVE_TRACK_LENGTH+DRIVE_TRACK_WIDTH*DRIVE_TRACK_WIDTH));

        public static final TrajectoryConstraint CONSTRAINT = new SwerveDriveKinematicsConstraint(KINEMATICS, MAX_WHEEL_SPEED);

        public static class CANChannels{
            public static final int FRONT_RIGHT_VELOCITY = 1;
            public static final int FRONT_RIGHT_ROTATION = 2;
            public static final int REAR_RIGHT_VELOCITY = 3;
            public static final int REAR_RIGHT_ROTATION = 4;
            public static final int REAR_LEFT_VELOCITY = 5;
            public static final int REAR_LEFT_ROTATION = 6;
            public static final int FRONT_LEFT_VELOCITY = 7;
            public static final int FRONT_LEFT_ROTATION = 8;
        }

        public static class CalibrationConstants{
            public static final double CALIBRATION_SPEED = -0.04;
            public static final int CALIBRATION_WAIT_MILLIS = 0;
            public static final int CALIBRATION_WAIT_NANOS = 10;

            public static final double FRONT_RIGHT_SWITCH_LOCATION = -4.122;
            public static final double REAR_RIGHT_SWITCH_LOCATION = -2.54;
            public static final double REAR_LEFT_SWITCH_LOCATION = -1.03;
            public static final double FRONT_LEFT_SWITCH_LOCATION = 0.598;

        }
    }
}
