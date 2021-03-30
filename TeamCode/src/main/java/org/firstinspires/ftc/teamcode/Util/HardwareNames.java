package org.firstinspires.ftc.teamcode.Util;

/**
 * This class holds all the hardware names for devices. Try to keep it in this class
 * in order to keep all the names the same
 *
 */
public class HardwareNames {


    public static class Drive {
        public static final String LEFT_FRONT = "lf";
        public static final String LEFT_BACK = "lb";
        public static final String RIGHT_FRONT = "rf";
        public static final String RIGHT_BACK = "rb";
    }

    public static class Odometry {
        public static final String LEFT_Y_ENCODER = "left_y_encoder";
        public static final String RIGHT_Y_ENCODER = "right_y_encoder";
        public static final String X_ENCODER = "x_encoder";
    }

    public static class WobbleGoal {
        public static final String WOBBLE_ARM = "wobble_arm";
        public static final String WOBBLE_CLAW = "wobble_claw";
    }

    public static class Hopper {
        public static final String HOPPER_SERVO = "hopper_servo";
        public static final String FLICKER = "flicker";
    }

    public static class Intake {
        public static final String INTAKE_MOTOR = "left_y_encoder";
    }

    public static class Sensors {
        public static final String LEFT_HUB_IMU = "imu";
        public static final String RIGHT_HUB_IMU = "imu 1";
    }

    public static class Shooter {
        public static final String SHOOTER_MOTOR_1 = "right_y_encoder";
        public static final String SHOOTER_MOTOR_2 = "shooter_motor_two";

        public static final String SHOOTER_ENCODER = SHOOTER_MOTOR_2;

        public static final String SHOOTER_ANGLE_SERVO = "shooter_angle_servo";
    }

    public static class Hubs {
        public static final String HUB_LEFT = "Expansion Hub 1";
        public static final String HUB_RIGHT = "Control Hub";
    }


}
