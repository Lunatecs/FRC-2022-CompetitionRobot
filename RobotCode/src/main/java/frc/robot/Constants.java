// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* */

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class DrivetrainConstants{
        public final static int LEFT_FRONT = 3;
        public final static int LEFT_BACK = 4;
        public final static int RIGHT_FRONT = 1;//Right motors replaced
        public final static int RIGHT_BACK = 2;

        public final static int PIGEON = 5;

        public final static double WHEEL_DIAMETER = 0.1016;
        public final static double TICKS = 2048.0;
        public final static double GEAR_REDUCTION = 7.09;
        public final static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public final static double METERS_PER_TICK = WHEEL_CIRCUMFERENCE / (TICKS * GEAR_REDUCTION);
    
        public final static double KS = 0.59; //0.55;
        public final static double KV = 2.36; //2.43;
        public final static double KA = 0.34; //0.15;
        public final static double KP = 2.45; //2.53;

        public final static double TRACK_WIDTH = 0.62;
        public final static DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
        
        public final static double MAX_VOLTAGE = 10.0;

        public final static double MAX_SPEED = 3.5;
        public final static double MAX_ACCELERATION = 3.0;

        public final static double RAMSETE_B = 2.0;
        public final static double RAMSETE_ZETA = 0.7;
    }

    public final class JoystickConstants{
        public final static int LEFT_X_AXIS = 0;
        public final static int LEFT_Y_AXIS = 1;
        public final static int RIGHT_X_AXIS = 4;
        public final static int RIGHT_Y_AXIS = 5;
        public final static int LEFT_JOYSTICK_BUTTON = 9;
        public final static int RIGHT_JOYSTICK_BUTTON = 10;

        public final static int GREEN_BUTTON = 1;
        public final static int RED_BUTTON = 2;
        public final static int YELLOW_BUTTON = 4;
        public final static int BLUE_BUTTON = 3;

        public final static int LEFT_TRIGGER = 2;
        public final static int RIGHT_TRIGGER = 3;
        public final static int LEFT_BUMPER = 5;
        public final static int RIGHT_BUMPER = 6;

        public final static int BACK_BUTTON = 7;
        public static final int START_BUTTON = 8;

        public final static int DRIVER_USB = 0;

        public final static int OPERATOR_USB = 1;

    }

    public final class ClimberConstants{
        public final static int FORWARD_CHANNEL_ARMS = 5;
        public final static int REVERSE_CHANNEL_ARMS = 11;
        public final static int FORWARD_CHANNEL_CLIMBER = 10;
        public final static int REVERSE_CHANNEL_CLIMBER = 3;

        public final static int WINCH_MOTOR_ID = 15; // Left
        public final static int WITCH_MOTOR_ID = 16; // Right

        public final static int LIMIT_SWITCH_PORT = 0; // Right
        public final static int LIMIT_SWITCH_PORT_1 = 1; // Left


    
    } 

    public final class IntakeConstants{
        public final static int FORWARD_MOTOR_ID = 10;
        public final static int BACK_MOTOR_ID = 5;

        public final static int FORWARD_CHANNEL_F = 6;
        public final static int REVERSE_CHANNEL_F = 12;

        public final static int FORWARD_CHANNEL_B = 7;
        public final static int REVERSE_CHANNEL_B = 8;
    }

    public final class TowerConstants{

        public final static int FRONT_TOWER_MOTOR_ID = 6;
        public final static int BACK_TOWER_MOTOR_ID = 14;
    }

    public static final class TurretConstants {
        public static final int Turret_ID = 18;

        public static final double MinSpeed = 0.05;
        public static final double ScanSpeed = 0.25;
        
        public static final int FwdMaxSensorPostion = 9500;//7500;//17000;
        public static final double FwdKp = 0.00015;
        public static final double FwdKd = 0.0;
        public static final double FwdKi = 0.0;
        
        public static final int BckMaxSensorPostion = -9500;//-7500;//-17000;
        public static final double BckKp = 0.00015;
        public static final double BckKd = 0.0;
        public static final double BckKi = 0.0;

        public final static int LIMIT_SWITCH_PORT_2 = 2; //turret limit switch
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_LEADER = 17;
        public static final int SHOOTER_FOLLOWER = 20;

        public static final int IDLE_SPEED = 5800;
    }

    public static final class TrackingConstants {
        public static final double kP = 0.03;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }
    
    public static final class LimelightConstants {
        public static final int forceOff_ID = 1;
        public static final int forceOn_ID = 3;
    }

    public static final class StopperConstants {
        public final static int FORWARD_CHANNEL = 4;
        public final static int REVERSE_CHANNEL = 9;
    }

    public static final class BallSensorConstants {
        public final static int TOP_SENSOR = 9;
        public final static int BOTTOM_SENSOR = 7;
    }

} 

