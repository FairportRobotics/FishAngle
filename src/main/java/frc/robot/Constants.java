package frc.robot;

import frc.robot.RobotContainer.GamePiece;

public class Constants {

    public static final Mode CURRENT_MODE = Mode.REAL;
    public static enum Mode {
      /** Running on a real robot. */
      REAL,
  
      /** Running a physics simulator. */
      SIM,
  
      /** Replaying from a log file. */
      REPLAY
    }

    public static final int DRIVER_XBOX_CONTROLLER_ID = 0;

    public static final int OPERATOR_XBOX_CONTROLLER_ID = 1;

    public static final int ARM_FALCON_ID = 20;
    public static final int ARM_POT_ID = 1;
    public static final int ARM_MAX_ROM_VALUE = 300000;
    public static final int ARM_MIN_ROM_VALUE = -0;

    public static final int WRIST_FALCON_ID = 50;
    public static final int WRIST_POT_ID = 0;
    public static final int WRIST_MAX_ROM_VALUE = 68000;
    public static final int WRIST_MIN_ROM_VALUE = 0;


    public static final int[] ARM_HIGH_POS = new int[2];
    public static final int[] ARM_STATION_POS = new int[2];
    public static final int[] ARM_HOME_POS = new int[2];
    public static final int[] ARM_MID_POS = new int[2];
    public static final int[] ARM_LOW_POS = new int[2];

    public static final int[] WRIST_HIGH_POS = new int[2];
    public static final int[] WRIST_STATION_POS = new int[2];
    public static final int[] WRIST_HOME_POS = new int[2];
    public static final int[] WRIST_MID_POS = new int[2];
    public static final int[] WRIST_LOW_POS = new int[2];

    static {

        ARM_HIGH_POS[GamePiece.CONE.valueOf()] = 208300;
        ARM_HIGH_POS[GamePiece.CUBE.valueOf()] = 208300;

        ARM_STATION_POS[GamePiece.CONE.valueOf()] = 215600;
        ARM_STATION_POS[GamePiece.CUBE.valueOf()] = 206400;

        ARM_MID_POS[GamePiece.CONE.valueOf()] = 171200;
        ARM_MID_POS[GamePiece.CUBE.valueOf()] = 171200;

        ARM_LOW_POS[GamePiece.CONE.valueOf()] = 30000;
        ARM_LOW_POS[GamePiece.CUBE.valueOf()] = 30000;

        ARM_HOME_POS[GamePiece.CONE.valueOf()] = 30000;
        ARM_HOME_POS[GamePiece.CUBE.valueOf()] = 30000;

        WRIST_HIGH_POS[GamePiece.CONE.valueOf()] = 46200;
        WRIST_HIGH_POS[GamePiece.CUBE.valueOf()] = 46200;

        WRIST_STATION_POS[GamePiece.CONE.valueOf()] = 59700;
        WRIST_STATION_POS[GamePiece.CUBE.valueOf()] = 58000;
        
        WRIST_MID_POS[GamePiece.CONE.valueOf()] = 38500;
        WRIST_MID_POS[GamePiece.CUBE.valueOf()] = 38500;

        WRIST_LOW_POS[GamePiece.CONE.valueOf()] = 39112;
        WRIST_LOW_POS[GamePiece.CUBE.valueOf()] = 39112;

        WRIST_HOME_POS[GamePiece.CONE.valueOf()] = 12000;
        WRIST_HOME_POS[GamePiece.CUBE.valueOf()] = 12000;

    }

    public static final int ARM_FOLDED_POS_VAL = 0;
    public static final int WRIST_FOLDED_POS_VAL = 0;

    public static final int ARM_BRAKE_SOLENOID = 4;

    public static final int GRIPPER_SOLENOID_OPEN_ID = 1;
    public static final int GRIPPER_SOLENOID_CLOSE_ID = 3;

    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5842;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4826;

    public static final int[] SWERVE_DRIVE_IDS = new int[4];
    public static final int[] SWERVE_STEER_IDS = new int[4];
    public static final int[] SWERVE_ENCODER_IDS = new int[4];
    public static final double[] SWERVE_OFFSETS = new double[4];

    static {

        // Front Left
        SWERVE_DRIVE_IDS[0] = 1;
        SWERVE_STEER_IDS[0] = 2;
        SWERVE_ENCODER_IDS[0] = 3;
        SWERVE_OFFSETS[0] = -Math.toRadians(282.832);

        // Front Right
        SWERVE_DRIVE_IDS[1] = 4;
        SWERVE_STEER_IDS[1] = 5;
        SWERVE_ENCODER_IDS[1] = 6;
        SWERVE_OFFSETS[1] = -Math.toRadians(241.787);

        // Back Left
        SWERVE_DRIVE_IDS[2] = 7;
        SWERVE_STEER_IDS[2] = 8;
        SWERVE_ENCODER_IDS[2] = 9;
        SWERVE_OFFSETS[2] = -Math.toRadians(155.918);

        // Back Right
        SWERVE_DRIVE_IDS[3] = 10;
        SWERVE_STEER_IDS[3] = 11;
        SWERVE_ENCODER_IDS[3] = 12;
        SWERVE_OFFSETS[3] = -Math.toRadians(107.842);

    }

    public static final int PH_CAN_ID = 18;

    public static final double PP_PID_P = CURRENT_MODE == Mode.REAL ? 1 : 0;
    public static final double PP_PID_I = CURRENT_MODE == Mode.REAL ? 0 : 0;
    public static final double PP_PID_D = CURRENT_MODE == Mode.REAL ? 0.1 : 0;

    public static final int ULTRASONIC_SENSOR_ID = 2;
    public static final double SUBSTATION_DISTANCE = 291;

}
