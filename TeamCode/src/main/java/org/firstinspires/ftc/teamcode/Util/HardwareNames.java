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

    public static class Sensors {
        public static final String LEFT_HUB_IMU = "imu";
        public static final String RIGHT_HUB_IMU = "imu 1";
    }

    public static class Shooter {
        public static final String SHOOTER_MOTOR_1 = "left_y_encoder";
        public static final String SHOOTER_MOTOR_2 = "x_encoder";

        public static final String SHOOTER_ENCODER = SHOOTER_MOTOR_2;

        public static final String SHOOTER_ANGLE_SERVO = "shooter angle servo";
    }

    public static class Hubs {
        public static final String HUB_LEFT = "Expansion Hub 10";
        public static final String HUB_RIGHT = "Expansion Hub 1";
    }


}
