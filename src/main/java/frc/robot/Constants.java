package frc.robot;

public class Constants {

    public static final int DRIVER_XBOX_CONTROLLER_ID = 0;

    public static final int OPERATOR_XBOX_CONTROLLER_ID = 1;
    public static final int OPERATOR_A_BTN = 1;

    public static final int ARM_FALCON_ID = 58;
    public static final int ARM_POT_ID = 1;
    public static final int ARM_MAX_POT_VALUE = 1886;
    public static final int ARM_MIN_POT_VALUE = 249;

    public static final int WRIST_FALCON_ID = 20;
    public static final int WRIST_POT_ID = 0;
    public static final int WRIST_MAX_POT_VALUE = 2500;
    public static final int WRIST_MIN_POT_VALUE = 2000;



    public static final int ARM_HIGH_POS_VAL_CUBE = 2048;
    public static final int WRIST_HIGH_POS_VAL_CUBE = 2048;

    public static final int ARM_MID_POS_VAL_CUBE = 1024;
    public static final int WRIST_MID_POS_VAL_CUBE = 1024;

    public static final int ARM_LOW_POS_VAL_CUBE = 512;
    public static final int WRIST_LOW_POS_VAL_CUBE = 512;



    public static final int ARM_HIGH_POS_VAL_CONE = 2048;
    public static final int WRIST_HIGH_POS_VAL_CONE = 2048;

    public static final int ARM_MID_POS_VAL_CONE = 1024;
    public static final int WRIST_MID_POS_VAL_CONE = 1024;

    public static final int ARM_LOW_POS_VAL_CONE = 512;
    public static final int WRIST_LOW_POS_VAL_CONE = 512;



    public static final int ARM_HOME_POS_VAL = 0;
    public static final int WRIST_HOME_POS_VAL = 0;

    public static final int GRIPPER_SOLENOID_OPEN_ID = 1;
    public static final int GRIPPER_SOLENOID_CLOSE_ID = 1;

    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.508;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.508;

    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_SWERVE_ID = 2;
    public static final int FRONT_LEFT_ENCODER_ID = 3;
    public static final double FRONT_LEFT_SWERVE_OFFSET = 104.062;

    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_SWERVE_ID = 5;
    public static final int FRONT_RIGHT_ENCODER_ID = 6;
    public static final double FRONT_RIGHT_SWERVE_OFFSET = -12.393;

    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_LEFT_SWERVE_ID = 8;
    public static final int BACK_LEFT_ENCODER_ID = 9;
    public static final double BACK_LEFT_SWERVE_OFFSET = -161.895;

    public static final int BACK_RIGHT_DRIVE_ID = 10;
    public static final int BACK_RIGHT_SWERVE_ID = 11;
    public static final int BACK_RIGHT_ENCODER_ID = 12;
    public static final double BACK_RIGHT_SWERVE_OFFSET = 291.533;

    public static final double WHEEL_RADIUS = .04;// in meters
    public static final double DRIVE_RATIO = 8.14;
    public static final double STEERING_RATIO = 7 / 150;
    public static final double DRIVE_ENCODER_RESOLUTION = 2048;
    public static final double ENCODER_TICKS_PER_METER = DRIVE_ENCODER_RESOLUTION * DRIVE_RATIO
            / (2 * Math.PI * WHEEL_RADIUS);

    public static final double MAX_ANG_VEL = 360;
    public static final double MAX_ANG_ACC = 3600;

    public static final double SWERVE_STEER_P = 0.2;
    public static final double SWERVE_STEER_I = 0.0;
    public static final double SWERVE_STEER_D = 0.1;

}
