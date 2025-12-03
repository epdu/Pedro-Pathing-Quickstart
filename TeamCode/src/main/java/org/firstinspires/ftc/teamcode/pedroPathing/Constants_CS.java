package org.firstinspires.ftc.teamcode.pedroPathing;
//package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.roadrunner.Pose2d;

public class Constants_CS {
    public static final int POSITION_X_IN = 25; // horizontal slides all the way in
    public static final int POSITION_B_EXTRUDE = 30;//horizontal slides  out //600
    public static final int POSITION_B_testing = 80;//horizontal slides  out //600
    public static final int POSITION_B_EXTRUDETransfer = 100;//horizontal slides  out //600 is too much
    public static final int POSITION_B_EXTRUDETransferC= 150;//horizontal slides  out //600 is too much
    //    public static final int POSITION_B_EXTRUDETransferC= 700;//horizontal slides  out //600 is too much
    public static final int POSITION_B_EXTRUDE_MORE = 265; //horizontal slides all the way out 800
    public static final int POSITION_A_BOTTOM = 0; //Vertical  slides all the way in
    public static final int POSITION_Y_LOWForTest = 300; // Vertical slides up //800 //1000 too high
    public static final int POSITION_Y_LOW = 400; // Vertical slides up //800 //1000 too high
    public static final int POSITION_Y_HIGH = 850;// 1270Vertical slides all the way up
    public static final int POSITION_Y_HIGHH = 2320;// 1300Vertical slides all the way up 1500 1900 still a little bit low//bucket
    public static final int POSITION_Y_HIGHHH = 1400;//Vertical slides all the way up
    public static final double SLIDE_POWER_H = 0.7; // Adjust as needed
    public static final double SLIDE_POWER_V = 0.85; // Adjust as needed
    public static final double IClawOpen = 0.25;
    public static final double IClawCloseLose = 0.48;//was 0.54
    public static final double IClawCloseInitialization = 0.48;
    public static final double IClawCloseTight = 0.54;
    public static final double IClawCloseSuperTight = 0.543;
    /*
    public static final double IClawCloseLose = 0.53;//was 0.54
    public static final double IClawCloseTight = 0.543;
    public static final double IClawCloseSuperTight = 0.546;
     */
    public static final double OClawOpen = 0.3;
    public static final double OClawCloseInitialization = 0.54;
    public static final double OClawCloseLose = 0.535;
    public static final double OClawCloseTight = 0.535;
    public static final double OClawCloseSuperTight = 0.5496;
    public static final double OArmTransferPosition = 0.999;
    public static final double OArmRearSpecimenPick = 0.15;
 //   public static final double OArmRearSpecimenPick = 0.06;//before 0126205
    public static final double OArmBucket = 0.365;
    public static final double OClawSpecimenChambers = 0.546;
    public static final double WristzyawRight = 0.15;
    public static final double WristzyawLeft = 0.5;
    public static final double WristxpitchDown = 0.58;
    public static final double WristxpitchIntermedia4PositionAdjust =0.19;
    public static final double  WristxpitchUp = 0.3;
    public static final double IArmLDown = 0.71;
    public static final double  IArmRDown = 0.71;
    public static final double IArmLDownForPick = 0.7295;
    public static final double  IArmRDownForPick = 0.7295;
    public static final double IArmLUp = 0.65;
    public static final double  IArmRUp = 0.65;
    public static final double OArmLInitialization = 0.97;
    public static final double OArmRInitialization = 0.97;
    public static final double OArmLInitializationhigher = 0.8;
    public static final double OArmRInitializationhigher = 0.8;

    public static final double SERVO_STEP = 0.01; // 每次调整的伺服步长
    public static final double servoPosition = 0.5;
    public static final double SLIDE_POWER = 0.8; // Adjust as needed
    public static final  float speedMultiplier = 0.5f;
    public static final float speedLimiter1 = 0.05f;
    public static final float speedLimiterFaster =0.75f;
    public static final float speedLimiterSlower = 0.45f;


//    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
//    //  applied to the drive motors to correct the error.
//    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
//    public static final double SPEED_GAIN  =  0.026  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    public static final double STRAFE_GAIN =  0.016 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//    public static final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//
//    public static final double MAX_AUTO_SPEED = 0.8;   //  Clip the approach speed to this max value (adjust for your robot)
//    public static final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    public static final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
//
//    public static final double MIN_AUTO_SPEED = 0.1;
//    public static final double MIN_AUTO_TURN = 0.1;
//    public static final double MIN_AUTO_STRAFE = 0.11;
//
//    public static final double RANGE_TOLERANCE = 0.15;
//    public static final double HEADING_TOLERANCE = 1.5;
//    public static final double YAW_TOLERANCE = 2.2;
//
//    public static final double relocTimeLimit = 1;
//
//    // Robot-field constants
//    public static final double DIST_FROM_BACKDROP = 8.5;
//    public static final double DIST_FROM_WALL = 8.75; //  this is how close the camera should get to the wall for starter stack (inches)
//
//    // Camera Constants
//    public static final int B100EXPOSUREMS = 1;
//    public static final int C920EXPOSUREMS = 1;
//
//    public static final double BOTTOM_CAM_X_OFFSET = 0.0; //right offset
//    public static final double BOTTOM_CAM_Y_OFFSET = 4.28496063; //forward offset
//    public static final double BOTTOM_CAM_Z_OFFSET = 2.0; //height offset
//    public static final Pose2d CAM_POSE = new Pose2d(BOTTOM_CAM_X_OFFSET, BOTTOM_CAM_Y_OFFSET, 0.0);
//    public static final int aprilTagSampleSize = 50;
//    public static final double relocMaxXDiff = 10.0;
//    public static final double relocMaxYDiff = 8.0;
//    public static final double relocMaxHeadingDiff = Math.toRadians(30);
//
//    // DL variables
//    public static final double DL_VELO = (2600.0) / 60 * 28;
//    public static final double DL_LAUNCH = 0.64;
//    public static final double DL_AWAY = 0.5;
//
//    // Multipliers
//    public static final double speedLimitLowerBound = 0.35;
//
//
//    // Vertical motion variables
//    public static final int SLIDE_GROUND = -0;
//    public static final int SLIDE_GROUNDLOW = 50;
//    public static final int SLIDE_LOW = 350;
//    public static final int SLIDE_LOWMED = 800;
//    public static final int SLIDE_MEDIUM = 1200;
//    public static final int SLIDE_MEDHIGH = 1600;
//    public static final int SLIDE_HIGH = 1970;
//    public static final int SLIDE_STACK1 = 0;
//    public static final int SLIDE_STACK2 = 70;
//    public static final int SLIDE_STACK3 = 120;
//    public static final int SLIDE_STACK4 = 170;
//    public static final int SLIDE_STACK5 = 210;
//
//    public static final int SLIDE_1 = 60;
//    public static final int SLIDE_2 = 300;
//    public static final int SLIDE_3 = 540;
//    public static final int SLIDE_4 = 760;
//    public static final int SLIDE_5 = 1000;
//    public static final int SLIDE_6 = 1240;
//    public static final int SLIDE_7 = 1480;
//    public static final int SLIDE_8 = 1680;
//    public static final int SLIDE_RIG = 1970;
//
//    public static final double V4B_RETRACTED_LEFT = 0.828;
//    public static final double V4B_GROUND_LEFT = 0.152 + 0.003;
//    public static final double V4B_LOW_LEFT = 0.34;
//    public static final double V4B_MEDIUM_LEFT = 0.34;
//    public static final double V4B_HIGH_LEFT = 0.43;
//
//    public static final double V4B_RETRACTED_RIGHT = 0.45-0.043;
//    public static final double V4B_GROUND_RIGHT = 0.355-0.043;
//    public static final double V4B_LOW_RIGHT = 0.54-0.043;
//    public static final double V4B_MEDIUM_RIGHT = 0.54-0.043;
//    public static final double V4B_HIGH_RIGHT = 0.54-0.043;
//
//    // Grabber variables
//    public static final double GRABBER_CLOSED = 0.61;
//    public static final double GRABBER_OPEN = 0.49;
//    public static final double GRABBER_WIDE = GRABBER_OPEN - 0.16;
//    public static final double GRABBER_DROP1 = 0.57;
//    public static final double GRABBER_DROP2 = 0.55;
//
//    // Pixel pick
//    public static final double PICK_SOFT = 0.0;
//    public static final double PICK_HARD = 0.87;
//
//    // Distance Sensor (on grabber) variables
//    public static final double disThreshold = 4.05;
//    public static final int FILTER_SIZE = 5;
//
//    // Purple v4b variables
//    public static final double PURPLE_RETRACT = 0.18;
//    public static final double PURPLE_DOWN = 0.69;
//    public static final double PURPLE_STACK6 = 0.44;
//    public static final double PURPLE_STACK5 = 0.50;
//    public static final double PURPLE_STACK4 = 0.56;
//    public static final double PURPLE_STACK3 = 0.62;
//    public static final double PURPLE_STACK2 = 0.68;
//    public static final double PURPLE_STACK1 = 0.70;
//
//    // Purple grabber variables
//    public static final double PURPLE_OPEN = 0.46;
//    public static final double PURPLE_OPENRETRACT = 0.50;
//    public static final double PURPLE_CLOSE = 0.56;
//
//    // Gamepad variables
//    public static final double triggerThreshold = 0.8;

    /**********************************************

     private static final int POSITION_X_IN = 0; // horizontal slides all the way in
     private static final int POSITION_B_EXTRUDE = 600;//horizontal slides  out //600
     private static final int POSITION_B_EXTRUDETransfer = 600;//horizontal slides  out //600 is too much
     private static final int POSITION_B_EXTRUDETransferC= 700;//horizontal slides  out //600 is too much
     private static final int POSITION_B_EXTRUDE_MORE = 800; //horizontal slides all the way out 800
     private static final int POSITION_A_BOTTOM = 0; //Vertical  slides all the way in
     private static final int POSITION_Y_LOW = 800; // Vertical slides up //800 //1000 too high
     private static final int POSITION_Y_HIGH = 1270;//Vertical slides all the way up
     private static final int POSITION_Y_HIGHH = 1300;//Vertical slides all the way up
     private static final double SLIDE_POWER_H = 0.4; // Adjust as needed
     private static final double SLIDE_POWER_V = 0.70; // Adjust as needed
     private static final double IClawOpen = 0.32;
     private static final double IClawCloseLose = 0.54;
     private static final double IClawCloseTight = 0.543;
     private static final double OClawOpen = 0.32;
     private static final double OClawCloseLose = 0.54;
     private static final double OClawCloseTight = 0.548;
     private static final double OArmTransferPosition = 0.99;
     private static final double OArmRearSpecimenPick = 0.06;
     private static final double OClawSpecimenChambers = 0.548;
     private static final double SERVO_STEP = 0.01; // 每次调整的伺服步长
     double servoPosition = 0.5;
     private static final double SLIDE_POWER = 0.8; // Adjust as needed
     public float speedMultiplier = 0.5f;
     public float speedLimiter = 0.05f;


     */
}
