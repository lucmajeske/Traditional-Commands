// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int GYRO_ID = 11;

        public static final int LEFT_LEADER_ID = 1;
        public static final int LEFT_FOLLOWER_ID = 2;
        public static final int RIGHT_LEADER_ID = 3;
        public static final int RIGHT_FOLLOWER_ID = 4;

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6.5);
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static final int COUNTS_PER_REVOLUTION = 4096;
        public static final double METERS_PER_PULSE = WHEEL_CIRCUMFERENCE / COUNTS_PER_REVOLUTION;

        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 100;
    }

    public static final class WristConstants {
        public static final double
        GOAL_ONE_HOME = 0.0,
        GOAL_TWO_INTAKE = 0.2331,
        GOAL_THREE_ALGAE_DEPOSIT = 0.1034,
        GOAL_FOUR_CORAL_LOW = 0.007,
        GOAL_FIVE_CORAL_HIGH = 0.007,
        GOAL_SIX_ALGAE_LOW = 0.1424,
        GOAL_SEVEN_ALGAE_HIGH = 0.3357,
        GOAL_EIGHT_BARGE_SHOOT = 0.2243;
            
        

        public static final int Left_Front_ID = 6;
        public static final int CURRENT_LIMIT = 60;
        public static final double NOMINAL_VOLTAGE = 10;
        public static final int Right_Front_ID = 8;

        // PID Constants 
        public static final double Wrist_P = 0; //5.25
        public static final double Wrist_I = 0; //0.25
        public static final double Wrist_D = 0; //1.25
    }
    public static final class ArmConstants {
        public static final int Left_Back_ID = 7;
        public static final int CURRENT_LIMIT = 80;
        public static final double NOMINAL_VOLTAGE = 10;
        public static final int Right_Back_ID = 9;

        // public static final int Arm_Encoder_ID = 7;
        // PID Constants
        public static final double Arm_P = 20.0; //5.0
        public static final double Arm_I = 0; //0.75
        public static final double Arm_D = 0; //0.75

        public static final double
            GOAL_ONE_HOME = 0.0,
            GOAL_TWO_INTAKE = 0.0185,
            GOAL_THREE_ALGAE_DEPOSIT = 0.0068,
            GOAL_FOUR_CORAL_LOW = 0.0966,
            GOAL_FIVE_CORAL_HIGH = 0.1307,
            GOAL_SIX_ALGAE_LOW = 0.0878,
            GOAL_SEVEN_ALGAE_HIGH = 0.2318,
            GOAL_EIGHT_BARGE_SHOOT = 0.2958;

    } 

    public static final class RollerConstants {
        public static final int ROLLER_MOTOR_ID = 10;
        public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
        public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
        public static final double ROLLER_EJECT_VALUE = 0.6;
    }

    public static final class OperatorConstants {
        public static final int DRIVER_LEFT_CONTROLLER_PORT = 0;
        public static final int DRIVER_RIGHT_CONTROLLER_PORT = 1;
        public static final int OPERATOR_CONTROLLER_PORT = 2;
    }
}
