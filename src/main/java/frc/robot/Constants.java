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

    public static final boolean DEBUG = false;

    public static class ClimbConstants{
        public static final int LENGTH_WINCH_LEADER_CHANNEL = 17;
        public static final int LENGTH_WINCH_FOLLOWER_CHANNEL = 18;

        public static final int LENGTH_ENCODER_A = 0;
        public static final int LENGTH_ENCODER_B = 1;


        public static final int ANGLE_WINCH_CHANNEL = 20;

        public static final double LENGTH_WINCH_GEARING = 1;
        public static final double LENGTH_WINCH_CIRCUMFRENCE = 1;

        public static final int ANGLE_WINCH_PD_SLOT = 0;
        public static final double WINCH_CURRENT_THRESHOLD = 1; //change later


       
        
        public static final double ARM_DROP_SPEED=-0.6; // was -0.2
        public static final double ARM_IN_SPEED = 1.0; // was 0.4

        public static final double WINCH_DESLACK_SPEED = 0.4;
        public static final double ARM_PULL_SPEED=-1;
        public static final double ARM_EXTEND_SPEED = 1;


        //public static final double LIFT_SLOW_HEIGHT =0;
        public static final double OVER_MID_BAR_HEIGHT = 3321;
        public static final double ON_BAR_HEIGHT = -200;
        public static final double OFF_BAR_HEIGHT = 700;
        public static final double FULL_EXTENSION_HEIGHT = 4527;

        public static final double PAST_NEXT_BAR_ANGLE = -98;
        public static final double ON_NEXT_BAR_ANGLE = -77;

        public static final double AUTOCLIMB_CHECK_PERIOD = 0.0002;


        public static final double MANUAL_PULL_SPEED = -1;
        public static final double MANUAL_EXTEND_SPEED = 1.0;//0.4
        public static final double MANUAL_TILT_SPEED = -0.4;

    }

    public final class LauncherContants{
        public static final int LEADER_CHANNEL = 13;
        public static final int FOLLOWER_CHANNEL = 14;
        public static final double ENCODER_POSITIONAL_CONVERSION = 1.0/3.0;
        public static final double PRIMARY_SECONDARY_RATIO = 0.5;
    }

    public final class VisionConstants{
        public static final int LIMELIGHT_SERVO_PWM_CHANNEL = 1;
        public static final double TARGET_HEIGHT_FEET = 8.6666;
    }

    public static class IntakeConstants{
        public static final int INTAKE_CHANNEL = 16;
        public static final int RAISE_LOWER_CHANNEL = 15;
        public static final int EXTEND_LIMIT_CHANNEL = 0;
        public static final int RETRACT_LIMIT_CHANNEL=1;

        public static final double spinSpeed = -1;

        public static final double EXTEND_LEVEL = 1;
        public static final double RETRACT_LEVEL = 1;

        public static final double INTAKE_EXTENDED_DISTANCE = 218; //change
        public static final double INTAKE_RETRACTED_DISTANCE = 0; //change?
        public static final double MID_LEVEL = 145.06;
        public static final double MAX_LEVEL_STABLE_OUTPUT = 0.05;

        public static final double MAX_RAISE_ACCEL = 0;
        public static final double MAX_RAISE_VEL = 0;
        public static final double MAX_LOWER_ACCEL = 0;
        public static final double MAX_LOWER_VEL = 0;
        
        public static final float REVERSE_SOFT_LIMIT = -138f;
    }

    public static class StagingConstants{
        public static final int MOTOR_CHANNEL = 19;
        public static final int SENSOR_CHANNEL = 16;
        public static final double IN_SPEED = 1;
        public static final double OUT_SPEED = -1;

        public static final double RUN_TIME = 0.1;

    }

    public static class DriveConstants{
        public static final double ROTATION_GEARING = 1.0/12.8;
        public static final double VELOCITY_GEARING = 1.0/6.75;
        public static final double WHEEL_CIRCUMFRENCE = Math.PI * 4*2.54 *0.01;

        public static final double DRIVE_TRACK_WIDTH = 0.43;
        public static final double DRIVE_TRACK_LENGTH = 0.73;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVE_TRACK_LENGTH/2, -DRIVE_TRACK_WIDTH/2), //fron right 
            new Translation2d(-DRIVE_TRACK_LENGTH/2, -DRIVE_TRACK_WIDTH/2), //rear right
            new Translation2d(-DRIVE_TRACK_LENGTH/2, DRIVE_TRACK_WIDTH/2), //rear left
            new Translation2d(DRIVE_TRACK_LENGTH/2, DRIVE_TRACK_WIDTH/2) //front left
          );
        
        
        public static final double MAX_WHEEL_SPEED = 4.38;

        public static final double MAX_TURN_SPEED = MAX_WHEEL_SPEED * (Math.sqrt(DRIVE_TRACK_LENGTH*DRIVE_TRACK_LENGTH+DRIVE_TRACK_WIDTH*DRIVE_TRACK_WIDTH));

        public static final TrajectoryConstraint CONSTRAINT = new SwerveDriveKinematicsConstraint(KINEMATICS, MAX_WHEEL_SPEED);
        public static final TrajectoryConfig CONFIG = new TrajectoryConfig(MAX_WHEEL_SPEED,2);

        public static class CANChannels{
            public static final int FRONT_RIGHT_VELOCITY = 2;
            public static final int FRONT_RIGHT_ROTATION = 1;
            public static final int REAR_RIGHT_VELOCITY = 8;
            public static final int REAR_RIGHT_ROTATION = 7;
            public static final int REAR_LEFT_VELOCITY = 6;
            public static final int REAR_LEFT_ROTATION = 5;
            public static final int FRONT_LEFT_VELOCITY = 4;
            public static final int FRONT_LEFT_ROTATION = 3;

            public static final int FRONT_RIGHT_CALIBRATION =9;
            public static final int REAR_RIGHT_CALIBRATION =12;
            public static final int REAR_LEFT_CALIBRATION =11;
            public static final int FRONT_LEFT_CALIBRATION =10;
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
