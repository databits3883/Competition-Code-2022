// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


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
        public static final int SPARK_MAX_CHANNEL = 13; //change later
        public static final double ENCODER_POSITIONAL_CONVERSION = 1/3;
    }

    public static class IntakeConstants{
        public static final int intakeMotorChannel = 0;
        public static final int RAISE_LOWER_CHANNEL = 1;
        public static final int EXTEND_LIMIT_CHANNEL = 0;
        public static final int RETRACT_LIMIT_CHANNEL=1;

        public static final double spinSpeed = 0.3;

        public static final double extendSpeed = 0.3;
        public static final double retractSpeed = -0.3;

        public static final double EXTENSION_TIMEOUT=3;
        public static final double RETRACT_TIMEOUT=3;

    }

    public static class StagingConstants{
        public static final int motorChannel = 0;
        public static final double motorSpeed = 0.5;
    }
}
