package frc.robot;

import frc.robot.RobotContainer.GamePiece;

public class Constants {

    public static final int DRIVER_XBOX_CONTROLLER_ID = 0;

    public static final int OPERATOR_XBOX_CONTROLLER_ID = 1;

    public static final int ARM_FALCON_ID = 58;
    public static final int ARM_POT_ID = 1;
    public static final int ARM_MAX_POT_VALUE = 1886;
    public static final int ARM_MIN_POT_VALUE = 249;

    public static final int WRIST_FALCON_ID = 20;
    public static final int WRIST_POT_ID = 0;
    public static final int WRIST_MAX_POT_VALUE = 2500;
    public static final int WRIST_MIN_POT_VALUE = 2000;


    public static final int[] ARM_HIGH_POS = new int[2];
    public static final int[] ARM_MID_POS = new int[2];
    public static final int[] ARM_LOW_POS = new int[2];

    public static final int[] WRIST_HIGH_POS = new int[2];
    public static final int[] WRIST_MID_POS = new int[2];
    public static final int[] WRIST_LOW_POS = new int[2];

    static {

        ARM_HIGH_POS[GamePiece.CONE.valueOf()] = 2048;
        ARM_HIGH_POS[GamePiece.CUBE.valueOf()] = 2048;

        ARM_MID_POS[GamePiece.CONE.valueOf()] = 1024;
        ARM_MID_POS[GamePiece.CUBE.valueOf()] = 1024;

        ARM_LOW_POS[GamePiece.CONE.valueOf()] = 512;
        ARM_LOW_POS[GamePiece.CUBE.valueOf()] = 512;

        WRIST_HIGH_POS[GamePiece.CONE.valueOf()] = 2048;
        WRIST_HIGH_POS[GamePiece.CUBE.valueOf()] = 2048;

        WRIST_MID_POS[GamePiece.CONE.valueOf()] = 1024;
        WRIST_MID_POS[GamePiece.CUBE.valueOf()] = 1024;

        WRIST_LOW_POS[GamePiece.CONE.valueOf()] = 512;
        WRIST_LOW_POS[GamePiece.CUBE.valueOf()] = 512;

    }

    public static final int ARM_HOME_POS_VAL = 0;
    public static final int WRIST_HOME_POS_VAL = 0;

    public static final int ARM_BRAKE_SOLENOID = 3;

    public static final int GRIPPER_SOLENOID_OPEN_ID = 1;
    public static final int GRIPPER_SOLENOID_CLOSE_ID = 1;

    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4826;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5842;

    public static final int[] SWERVE_DRIVE_IDS = new int[4];
    public static final int[] SWERVE_STEER_IDS = new int[4];
    public static final int[] SWERVE_ENCODER_IDS = new int[4];
    public static final double[] SWERVE_OFFSETS = new double[4];

    static {

        // Front Left
        SWERVE_DRIVE_IDS[0] = 1;
        SWERVE_STEER_IDS[0] = 2;
        SWERVE_ENCODER_IDS[0] = 3;
        SWERVE_OFFSETS[0] = 104.062;

        // Front Right
        SWERVE_DRIVE_IDS[1] = 4;
        SWERVE_STEER_IDS[1] = 5;
        SWERVE_ENCODER_IDS[1] = 6;
        SWERVE_OFFSETS[1] = -12.393;

        // Back Left
        SWERVE_DRIVE_IDS[2] = 7;
        SWERVE_STEER_IDS[2] = 8;
        SWERVE_ENCODER_IDS[2] = 9;
        SWERVE_OFFSETS[2] = -161.895;

        // Back Right
        SWERVE_DRIVE_IDS[3] = 10;
        SWERVE_STEER_IDS[3] = 11;
        SWERVE_ENCODER_IDS[3] = 12;
        SWERVE_OFFSETS[3] = 291.533;

    }

    public static final Mode CURRENT_MODE = Mode.SIM;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,
  
      /** Running a physics simulator. */
      SIM,
  
      /** Replaying from a log file. */
      REPLAY
    }

}
