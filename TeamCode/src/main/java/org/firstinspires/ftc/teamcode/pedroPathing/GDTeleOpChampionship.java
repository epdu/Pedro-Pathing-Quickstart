package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "GD Pennsylvania FTC Championship V1 02192026")

public class GDTeleOpChampionship extends LinearOpMode {






    // 已有的硬件和常量定义...
    private static final double VELOCITY_TOLERANCE = 30; // RPM容差，可根据测试调整
    // 状态变量
    private boolean isShooterAtSpeed = false;
    private boolean wasShooterAtSpeed = false; // 用于检测状态变化
    private boolean fireRequested = false;
    // LED颜色常量（根据你的LED库调整）
    private final String LED_COLOR_READY = "GREEN";
    private final String LED_COLOR_ACCELERATING = "YELLOW";
    private final String LED_COLOR_OFF = "RED";
    /////////////////////////////////pretty goood for close shoot /////////////////////////// 1300
    private static final double Med_SHOOTER_TARGET_RPM = 1200;  // from 1200-1150 1666 still big 1866 kind of good for far， but a little bit too big
    public float DriveTrains_ReducePOWER=1f;
    public float DriveTrains_smoothTurn=1f;
    public float  intakePowerIntake=0.9f;//push blocker too much from 99-90
    public float  intakePowerShoot=0.95f;
    public float  intakePowerDump=-0.65f;
    public float  intakePowerOff=0.0f;
    public float  ShooterMotorShootFar=0.95f;
    public float  ShooterMotorShootMed=-0.8f;
    public float  ShooterMotorShootClose=-0.8f;
    public float  ShooterMotorHold=-0.2f;
    public float  ShooterMotorClean=-0.8f;
    public float  ShooterMotorOff=0.0f;
    public static final double blockageblockposition=0.10; //for auto
    public static final double blockagereleaseposition=0.18;
    public double servoPosition=0.5;
    public static final double SERVO_STEP=-0.005;
    public static final double blockageblockTele=0.1; // from .18 -0.1 for tele
    public static final double blockagereleaseTele=0.24;
    ///////////////turret///////////
    private Pose robotPose;
    private Pose pose;

    private double lastHeading = 0;
    private double turretAngle = Math.PI / 2.0;
    private double turretSetpoint = 0.0;
    private double turretPower = 0.0;
    private Follower follower;
    private PIDFController pidController;


    // Run-to-position mode flag
    private boolean rtp;
    // Current power applied to servo
    private double power;
    // Maximum allowed power
    private double maxPower;
    // Direction of servo movement
    private RTPAxon.Direction direction;
    // Last measured angle
    private double previousAngle;
    // Accumulated rotation in degrees
    private double totalRotation;
    // Target rotation in degrees
    private double targetRotation;

    // PID controller coefficients and state
    private double kP;
    private double kI;
    private double kD;
    private double integralSum;
    private double lastError;
    private double maxIntegralSum;
    private ElapsedTime pidTimer;

    // Initialization and debug fields
    public double STARTPOS;
    public int ntry = 0;
    public int cliffs = 0;
    public double homeAngle;

//    private final Gamepad gamepad1;
//    public static final double GEAR_RATIO = .5;
//    private DriveSubsystem driveSubsystem;
//    private static TurretSubsystem instance;
    /// ////////////////////////////////////////////////////////////////
    public static final double HoodArmfarposition=0.3;
    public static final double HoodArmcloseposition=0.35;
    public static final double HoodArmPositionInit = 0.1;
    public static final double HoodArmPositionCloseShoot = 0.3;
    public static final double HoodArmPositionMedShoot = 0.2;
    public static final double HoodArmPositionFarShoot = 0.1;
    // RPM = (TPS * 60秒) / 每转ticks数  return (tps * 60.0) / ticksPerRevolution;  28*13.7
    private static final double Close_SHOOTER_TARGET_RPM = 800;//  400RPM---2,557.33333333333333
    private static final double Far_SHOOTER_TARGET_RPM = 2237;  //  350RPM---2237
    public static double toleranceforShoot = 100;        // 转速容差
    //  private static final double Med_SHOOTER_TARGET_RPM = 1300;   //1598 white tri a little bit too far//  250RPM---1586.67
    //    private static final double Med_SHOOTER_TARGET_RPM = 1300;   //1598 white tri a little bit too far//  250RPM---1586.67
    // use rpm as speed, the real name should be speed = 1300      //1300
    //  1000RPM---6346.67
    //  600RPM---3808
    //  500RPM---3173.3

    HardwareQualifier robot = new HardwareQualifier();
//    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    //    private final Robot solverrobot = Robot.getInstance();
    //    public String fieldOrRobotCentric = "robot";
    public String fieldOrRobotCentric = "field";
    private double powerMultiplier = 0.9;
    boolean move = false;
    int controlMode = 1;

    private ElapsedTime imuResetTimer = new ElapsedTime();
    private boolean imuResetInCooldown = false;
    private static final long IMU_RESET_COOLDOWN_MS = 300; // 1秒冷却时间
    ButtonHandler dpadDownHandler = new ButtonHandler();
    ButtonHandler dpadUpHandler = new ButtonHandler();
    ButtonHandler dpadLeftHandler = new ButtonHandler();
    ButtonHandler dpadRightHandler = new ButtonHandler();
    ButtonHandler leftBumperHandler = new ButtonHandler();
    ButtonHandler rightBumperHandler = new ButtonHandler();
    ButtonHandler gamepad1XHandler = new ButtonHandler();
    ButtonHandler gamepad1BHandler = new ButtonHandler();
    ButtonHandler gamepad1YHandler = new ButtonHandler();
    ButtonHandler gamepad1AHandler = new ButtonHandler();
    ButtonHandler gamepad1BackHandler = new ButtonHandler();
    private volatile boolean isRunning = true;
    ElapsedTime delayTimer = new ElapsedTime();
    // 计时器
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

//        initializeTurret();
        initShooterPIDF();
//        robot.alliance = Alliance.BLUE;
        robot.alliance = Alliance.RED;
        follower = Constants.createFollower(hardwareMap);

        // 设置初始位置（通常在 opMode 的初始化阶段）
        Pose startPose = new Pose(70, 70, 0);  // 或其他起始坐标
        //auto off line 112, 92.25

        follower.setStartingPose(startPose == null ? new Pose() : startPose);
        follower.update();

        pidController = new PIDFController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.kF
        );


//        if (gamepad1.dpad_right) {
//            solverrobot.turret.setTurret(Turret.TurretState.GOAL_LOCK_CONTROL, 0);
//        } else if(gamepad1.dpad_left){
//            solverrobot.turret.setTurret(Turret.TurretState.OFF, 0);
//        }
//        robot.axonTurretArmL.setTargetRotation(0);
//        robot.axonTurretArmL.setMaxPower(0.5);
//        robot.axonTurretArmL.setPidCoeffs(0.000002, 0.0000, 0.000);
//        robot.axonTurretArmL.setTargetRotation(0);
//        robot.axonTurretArmL.setMaxPower(0.5);  // Limit max power to 50%
//        robot.axonTurretArmR.setMaxPower(0.5);  // Limit max power to 50%
//        robot.axonTurretArmR.setPidCoeffs(0.02, 0.0005, 0.0025);




        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard Dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = Dashboard.getTelemetry();
        // 设置 Dashboard 更新频率
        Dashboard.setTelemetryTransmissionInterval(100); // 100ms 更新一次

//        telemetry.addData("setStartingPose", startPose.getX());
//        telemetry.addData("setStartingPose", startPose.getY());
//        telemetry.addData("setStartingPose", startPose.getHeading());
//        telemetry.addData("follower.getPose().getX()", follower.getPose().getX());
//        telemetry.addData("follower.getPose().getX()", follower.getPose().getY());
//        telemetry.addData("follower.getPose().getX()", follower.getPose().getHeading());
//        telemetry.addData("follower", "OK");
        telemetry.addData("Start Pose", "X:%.1f Y:%.1f H:%.1f",
                startPose.getX(), startPose.getY(), startPose.getHeading());

        telemetry.update();

        waitForStart();
        runtime.reset();
        delayTimer.reset();

        while (opModeIsActive()) {

            updateDrivetrain_FieldCentric();
            handleIMUReset();
//            updateDrivetrain_RobotCentric();
            updateIntake();
            updateShooter();
            updateTelemetry();
            checkShooterVelocity();
            updateLEDs();
            updateHood();
            updateBlockage();
//            updateAutoAim();

//             turp();
            // Always update measured position
//            robotPose = follower.getPose();
//            setTurretPosition(0);

            turretAngle = getPosition();
            turretupdate();
//            setTurretPosition(turretSetpoint);
            robot.axonTurretArmL.update();
            robot.axonTurretArmR.update();
            telemetry.update();
            sleep(20);

        } //end of while loop

    } //end of run mode

/////////////////////////////////////////////methods/////////////////////////

    ///////////////////////////////////////seattlesolvers/////////////////////////

//    public void updateAutoAim(){
//        if(gamepad1.dpad_down){
//            solverrobot.launcher.setFlywheel(0, false);
//        } else if(gamepad1.dpad_up){
//            new ClearLaunch(true);
//            solverrobot.readyToLaunch = true;
//            solverrobot.launcher.setActiveControl(true);
//            solverrobot.launcher.setRamp(true);
//        }
//    }




/// ////////////////////////////////////////////////
private void turretupdate() {

    if (gamepad1.dpad_up) {
        turretSetpoint = findPosition();
        robot.axonTurretArmL.setTargetRotation(45);
        robot.axonTurretArmR.setTargetRotation(45);
        ////////////////////////////
//        robot.axonTurretArmL.setTargetRotation(Math.toDegrees(turretSetpoint));
//        robot.axonTurretArmR.setTargetRotation(Math.toDegrees(turretSetpoint));
        /// ///////////////////////////
//        robot.axonTurretArmL.changeTargetRotation(Math.toDegrees(turretSetpoint));
//        robot.axonTurretArmR.changeTargetRotation(Math.toDegrees(turretSetpoint));
//        axon.setTargetRotation(90);    // Move to 90 degrees absolute
//        axon.changeTargetRotation(45); // Move 45 degrees from current position
//        robot.axonTurretArmL.changeTargetRotation(45);
//        robot.axonTurretArmR.changeTargetRotation(45);
        telemetry.addData("turretSetpoint ", Math.toDegrees(turretSetpoint));

    } else if (gamepad1.dpad_down)  {
        turretSetpoint = 0.0;
//        telemetry.addData("X not", "NO X");
    }
}



    public double findPosition() {
        double x, y;
        double robotHeading;
        double overallAngle;

        if (robot.alliance == Alliance.BLUE) {
            x = follower.getPose().getX();
            y = 144 - follower.getPose().getY();

            robotHeading = follower.getPose().getHeading();
            overallAngle = Math.PI - Math.atan2(y, x);
            telemetry.addData("Alliance.BLUE",Alliance.BLUE);
        } else if (robot.alliance == Alliance.RED) {
            x = 144 - follower.getPose().getX();
            y = 144 - follower.getPose().getY();
            telemetry.addData("Alliance.RED", Alliance.RED);
            robotHeading = follower.getPose().getHeading();
            overallAngle = Math.atan2(y, x);
            telemetry.addData("robotHeading", robotHeading);
            telemetry.addData("overallAngle", overallAngle);

        } else {
            return 0.0;
        }

        double target = overallAngle - robotHeading;
        //代替power PIDF 使用 angle PID


        if (target < -Math.PI / 2.0 || target > Math.PI / 2.0) {


            return 0.0;
        }
        telemetry.addData("target ", target);
        telemetry.addData("tMath.toDegrees(target) ",  Math.toDegrees(target));
        return target;
    }

    public double getPosition() {
        if (robot.revEncoder == null) {
            telemetry.addData("Warning", "revEncoderForTurret is null in getPosition()");
            return 0.0;
        }
        int ticksPerRev = 8192;//16384
        double revolutions = (double) robot.revEncoder.getCurrentPosition() / ticksPerRev;
        telemetry.addData(" -revolutions * 2 * Math.PI * TurretConstants.GEAR_RATIO ",  -revolutions * 2 * Math.PI * TurretConstants.GEAR_RATIO);
        return -revolutions * 2 * Math.PI * TurretConstants.GEAR_RATIO;

    }


    public double getCurrentAngleOfTurret() {
        if (robot.revEncoder == null) {
            telemetry.addData("Warning", "revEncoderForTurret is null in getCurrentAngleofTurret()");
            return 0.0;
        }

        return ( robot.revEncoder.getCurrentPosition()/16384) * (direction.equals(RTPAxon.Direction.FORWARD) ? -360 : 360);
    }


public void setTurretPosition(double pos) {
    turretPower = - pidController.calculate(getPosition(), pos);
//    robot.servoTurretArmL.setPower(turretPower);
//    robot.servoTurretArmR.setPower(turretPower);

    setTurretPower(turretPower);
}

    public void setTurretPower(double power) {
        double clampedPower = Math.max(-0.5, Math.min(0.5, power)); // 可以是-1  +1 区间

        turretPower = clampedPower;
//        robot.servoTurretArmL.setPower(clampedPower);
//        robot.servoTurretArmR.setPower(clampedPower);

//        telemetry.addData("Final Power", turretPower);
//        telemetry.addData(" BturretPower ",  turretPower);
//        telemetry.addData(" turretPower ",  power);
//        turretPower = power;
//        telemetry.addData(" AturretPower ",  turretPower);
//        telemetry.addData(" turretPower ",  power);
//        robot.servoTurretArmL.setPower(power);
//        robot.servoTurretArmR.setPower(power);



    }
    public Pose getPose() {
        return new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading() + lastHeading, PedroCoordinates.INSTANCE);
    }
    public void stopturret() { // stop --- stopturret
        turretPower = 0.0;
        robot.servoTurretArmL.setPower(0.0);
        robot.servoTurretArmR.setPower(0.0);
    }

//public enum TurretState {
//    GOAL_LOCK_CONTROL,
//    ANGLE_CONTROL,
//    OFF,
//}
//    public static PIDFCoefficients TURRET_LARGE_PIDF_COEFFICIENTS = new PIDFCoefficients(0.01,0.000, 0.00044, 0.0); // Coefficients for radians
//private Pose2d turretPose = null;
//    public static TurretState turretState = TurretState.GOAL_LOCK_CONTROL;

//    PIDController turretController = new PIDController(0.015, 0, 0);
//    public PIDFController turretController = new PIDFController(TURRET_LARGE_PIDF_COEFFICIENTS);
    public static double targetVel = 0;
//    public class turretController {
//
//        double kP;
//        double kI;
//        double kD;
//
//        double lastError = 0;
//        double integral = 0;
//
//        public turretController(double p, double i, double d) {
//            kP = p;
//            kI = i;
//            kD = d;
//        }
//
//        public double calculate(double current, double target) {
//
//            double error = target - current;
//
//            integral += error;
//
//            double derivative = error - lastError;
//
//            lastError = error;
//
//            double output = (kP * error) + (kI * integral) + (kD * derivative);
//
//            return output;
//        }
//    }
//

/////////////////////////////

// Main update loop: updates rotation, computes PID, applies power update ----updateTurret
public synchronized void updateTurret() {
    double currentAngle = getCurrentAngleOfTurret();
    double angleDifference = currentAngle - previousAngle;

    // Handle wraparound at 0/360 degrees
    if (angleDifference > 180) {
        angleDifference -= 360;
        cliffs--;
    } else if (angleDifference < -180) {
        angleDifference += 360;
        cliffs++;
    }

    // Update total rotation with wraparound correction
    totalRotation = currentAngle - homeAngle + cliffs * 360;
    previousAngle = currentAngle;

    if (!rtp) return;

    double dt = pidTimer.seconds();
    pidTimer.reset();

    // Ignore unreasonable dt values
    if (dt < 0.001 || dt > 1.0) {
        return;
    }

    double error = targetRotation - totalRotation;

    // PID integral calculation with clamping
    integralSum += error * dt;
    integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

    // Integral wind-down in deadzone
    final double INTEGRAL_DEADZONE = 2.0;
    if (Math.abs(error) < INTEGRAL_DEADZONE) {
        integralSum *= 0.95;
    }

    // PID derivative calculation
    double derivative = (error - lastError) / dt;
    lastError = error;

    // PID output calculation
    double pTerm = kP * error;
    double iTerm = kI * integralSum;
    double dTerm = kD * derivative;

    double output = pTerm + iTerm + dTerm;

    // Deadzone for output
    final double DEADZONE = 0.5;
    if (Math.abs(error) > DEADZONE) {
        double power = Math.min(maxPower, Math.abs(output)) * Math.signum(output);
        robot.servoTurretArmL.setPower(power);
        robot.servoTurretArmR.setPower(power);
    } else {
        robot.servoTurretArmL.setPower(0);
        robot.servoTurretArmR.setPower(0);
    }
}

/// ///////////////////////

// Initialization logic for servo and encoder  initialize ----initializeTurret
private void initializeTurret() {
    robot.servoTurretArmL.setPower(0);
    robot.servoTurretArmR.setPower(0);
    try {
        Thread.sleep(50);
    } catch (InterruptedException ignored) {
    }

    // Try to get a valid starting position
    do {
        STARTPOS = getCurrentAngleOfTurret();
        if (Math.abs(STARTPOS) > 1) {
            previousAngle = getCurrentAngleOfTurret();
        } else {
            try {
                Thread.sleep(50);
            } catch (InterruptedException ignored) {
            }
        }
        ntry++;
    } while (Math.abs(previousAngle) < 0.2 && (ntry < 50));

    totalRotation = 0;
    homeAngle = previousAngle;

    // Default PID coefficients
    kP = 0.015;
    kI = 0.0005;
    kD = 0.0025;
    integralSum = 0.0;
    lastError = 0.0;
    maxIntegralSum = 100.0;
    pidTimer = new ElapsedTime();
    pidTimer.reset();

    maxPower = 0.25;
    cliffs = 0;
}
    // endregion


/// ////////////////////////////////////////////////////
    public double normA(double angle) {
        angle %= 360;
        if (angle < -180) angle += 360;
        else if (angle > 180) angle -= 360;
        return angle;
    }


//public double turp() {
//
////        robotPose = new Pose(70, 70, 0);
//      Pose currentPose = follower.getPose();
//    // 或者直接使用，不存储到成员变量
////        Pose currentPose = robot.drive.getPose();
////        double dx = goalX - currentPose.getX();
////        double dy = goalY - currentPose.getY();
//
//    int ticks = 16384;
////    int ticks = 8192;
////    only for RED
//
//    double goalX = 144;
//    double goalY = 144;
//    double dx = (currentPose.getX()) -goalX;
//    double dy = goalY - (currentPose.getY());
//
//    double goalHeadingField = Math.atan2(dy, dx);
//    double goalHeadingFieldDegrees = Math.toDegrees(goalHeadingField);
//
//    double robotHeading =currentPose.getHeading();
////    robotPose.getHeading();follower.getPose()
//    double robotHeadingDegrees = Math.toDegrees(robotHeading);
//
//    double turretTargetAngle = 180 - goalHeadingFieldDegrees - robotHeadingDegrees;
//    telemetry.addData("turretTargetAngle", turretTargetAngle);
////    double turretAngle = (robot.encoderTurret.getCurrentPosition())/ticks;
//    double turretAngle = getCurrentAngle();
//
////    double target = normA(turretTargetAngle);
//    double target = normA(turretTargetAngle);
///// ////////////
//    target = interpolateAngle(target);
//    telemetry.addData("target", target);
//    telemetry.addData("urretAngle", turretAngle);
//    /// ////////
////    if (target > 150) {
////        target = 150;
////    } else if (target < -150) {
////        target = -150;
////    }
//
//    double turretPower = (turretController.calculate(turretAngle, target));
////    double turretPower = (calculate(turretAngle, target));
//    double error = target - turretAngle;
//    double tolerance = 2;
//
//    if (Math.abs(error) < tolerance) {
//        turretPower = 0;
//    }
////    robot.servoTurretArmL.setPower(turretPower);
////    robot.servoTurretArmR.setPower(turretPower);
//    robot.servoTurretArmL.setPower(target - turretAngle);
//    robot.servoTurretArmR.setPower(target - turretAngle);
//    robot.axonTurretArmL.setTargetRotation(overallAngle);
//    robot.axonTurretArmR.setTargetRotation(overallAngle);
//    robot.axonTurretArmL.changeTargetRotation(target - turretAngle);
//    robot.axonTurretArmR.changeTargetRotation(target - turretAngle);
//
//    telemetry.addData("encoderTurret .getCurrentPosition()", robot.encoderTurret.getCurrentPosition());
//    telemetry.addData("encoderTurret target angle-turret angle", target-turretAngle);
//    telemetry.update();
//    return goalX;
//}
//    private double normaltoservopower(double errorofangle){
//
//    }

    private double interpolateAngle(double angle) {
        angle = AngleUnit.normalizeDegrees(angle);
        if (angle > 165) {
            angle = 165;
        } else if (angle < -155) {
            angle = -155;
        }

        return (angle + 155) / 320;

    }












    /// //////////////////////////////////////////////////////////////////////////
    public void updateIntake() {
        // 手柄控制拾取电机
        if (gamepad1.left_trigger > 0.1) {
            /// ///////////////////////////////////for debug//////////////////////
            robot.BlockageArmL.setPosition(blockageblockposition);
            robot.BlockageArmR.setPosition(blockageblockposition);

            robot.IntakeMotorL.setPower(intakePowerIntake);
            robot.IntakeMotorR.setPower(intakePowerIntake);
            /// ///////////////////////////////////for debug//////////////////////
//                robot.axonTurretArmL.setTargetRotation(45);// ((96/20)*35/110)
//                robot.axonTurretArmR.setTargetRotation(45);
//            robot.axonTurretArmL.changeTargetRotation(45);
//            robot.axonTurretArmR.changeTargetRotation(45);

//y = 0.45452 -0.00677*H43^1 + -9.37853E-4*H43^2 -4.69942E-5*H43^3 + -1.24595E-6*H43^4 -1.74452E-8*H43^5 + -1.05357E-10*H43^6
            delayTimer.reset();
            while (delayTimer.milliseconds() < 300 && opModeIsActive()) {
                // Other tasks can be processed here
            } // 防止快速连击导致模式快速切换

        } else if (gamepad1.left_bumper) {
            // 反转（吐出）
            robot.IntakeMotorL.setPower(intakePowerDump);
            robot.IntakeMotorR.setPower(intakePowerDump);

            delayTimer.reset();
            while (delayTimer.milliseconds() < 300 && opModeIsActive()) {
                // Other tasks can be processed here
            } // 防止快速连击导致模式快速切换

        } else if (gamepad1.y)  {
            // 停止
            stopIntake();
        }
    }

    public void updateShooter() {

        if (gamepad1.right_trigger > 0.1) {
            checkShooterVelocity();
            if (!robot.MasterShooterMotorL.isBusy()){
                startShooter();
                robot.BlockageArmL.setPosition(blockagereleaseposition);
                robot.BlockageArmR.setPosition(blockagereleaseposition);
            }
        }
//        if (gamepad1.right_bumper && isShooterAtSpeed && !fireRequested)
        if (gamepad1.right_bumper) {
            fireRequested = true;
            robot.IntakeMotorL.setPower(intakePowerShoot);
            robot.IntakeMotorR.setPower(intakePowerShoot);
        }
        if (gamepad1.b) {
            stopShooter();
            robot.IntakeMotorL.setPower(intakePowerOff);
            robot.IntakeMotorR.setPower(intakePowerOff);
            robot.BlockageArmL.setPosition(blockageblockposition);
            robot.BlockageArmR.setPosition(blockageblockposition);
            fireRequested = false;
        }

/// ////////////////////////////////////////




//        if (gamepad1.x) {
//            turp();
//        } else {
//            robot.servoTurretArmL.setPower(0);
//            robot.servoTurretArmR.setPower(0);
//        }




/// /////////////////////////////////////////////


        if (!gamepad1.right_bumper && !robot.MasterShooterMotorL.isBusy()) {
            isShooterAtSpeed = false;
            fireRequested = false;
        }
        updateTelemetry();
    }

    public static class ShooterPIDFConfig {
        public static double kP = 0.0;   // 比例增益0.10.350.651.0655
        public static double kI = 0.0;      // 积分增益
        public static double kD = 0.0;      // 微分增益
        public static double kF = 0.0001;      // 前馈增益0.050.08
        public static double targetRPM =Med_SHOOTER_TARGET_RPM; // 目标转速
        public static double tolerance = 30;
        /////////////////////We have the exact same setup (although our wheel may be built differently).
        //////////////////before add more weight
//        public static double kP = 100;     // 比例增益0.10.350.651.0655
//        public static double kI = 0.0;      // 积分增益
//        public static double kD = 0.0;      // 微分增益
//        public static double kF = 17;      // 前馈增益0.050.08
    }

    private void initShooterPIDF() {
        // 初始化时调用一次
        if (robot.MasterShooterMotorL instanceof DcMotorEx) {
            DcMotorEx shooter = (DcMotorEx) robot.MasterShooterMotorL;
            DcMotorEx shooterR = (DcMotorEx) robot.SlaveShooterMotorR;
            // 直接使用setVelocity，它会使用已配置的PIDF
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // 设置PIDF参数
            PIDFCoefficients pidf = new PIDFCoefficients(
                    ShooterPIDFConfig.kP,
                    ShooterPIDFConfig.kI,
                    ShooterPIDFConfig.kD,
                    ShooterPIDFConfig.kF
            );
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            telemetry.addData("Shooter PIDF", "Initialized");
        }
    }

    /// ///////////need fix
    private void updateHood() {
        if (gamepad1.dpad_down) {
            robot.HoodArmL.setPosition(HoodArmfarposition);
        }  else if(gamepad1.dpad_up) {
            robot.HoodArmL.setPosition(HoodArmcloseposition);
        } // 防止快速连击导致模式快速切换

    }
    /// /////////////////need work
    private void updateBlockage() {
        if (gamepad1.dpad_left) {
//            robot.BlockageArm.setPosition((blockageblockTele));
            robot.BlockageArmL.setPosition((blockageblockposition));
            robot.BlockageArmR.setPosition((blockageblockposition));
        }  else if(gamepad1.dpad_right) {
//            robot.BlockageArm.setPosition((blockagereleaseTele));
            robot.BlockageArmL.setPosition((blockagereleaseposition));
            robot.BlockageArmR.setPosition((blockagereleaseposition));

        } // 防止快速连击导致模式快速切换

    }


    private void startShooter() {
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        robot.axonTurretArmL.setTargetRotation(0);
        robot.axonTurretArmR.setTargetRotation(0);
        if (robot.MasterShooterMotorL instanceof DcMotorEx) {
            DcMotorEx shooter = (DcMotorEx) robot.MasterShooterMotorL;
            DcMotorEx shooterR = (DcMotorEx) robot.SlaveShooterMotorR;
            // 直接使用setVelocity，它会使用已配置的PIDF
//            shooter.setVelocity(ShooterPIDFConfig.targetRPM)
            shooter.setVelocity(Math.abs(ShooterPIDFConfig.targetRPM));
            shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetRPM));
        }

    }

    private void moveTurret() {
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        if (robot.MasterShooterMotorL instanceof DcMotorEx) {/// ////////////////////
            robot.axonTurretArmL.setTargetRotation(90);// ((96/20)*35/110)
            robot.axonTurretArmR.setTargetRotation(90);

        }

    }

    private void checkShooterVelocity() {

        double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());
        double targetVelocity = ShooterPIDFConfig.targetRPM;
        double tolerance = ShooterPIDFConfig.tolerance;

        // 检查是否在容差范围内
        if (Math.abs(Math.abs(currentVelocity) - targetVelocity) <= toleranceforShoot) {
            isShooterAtSpeed = true;
        } else {
            isShooterAtSpeed = false;
            fireRequested = false;
        }

    }

    private void updateTelemetry() {

        double shooterVelocity = robot.MasterShooterMotorL.getVelocity();
        double shooterPower = robot.MasterShooterMotorL.getPower();
        double shooterCurrent = robot.MasterShooterMotorL.getCurrent(CurrentUnit.AMPS); // 如果有电流传感器
        double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());
        double targetVelocity = ShooterPIDFConfig.targetRPM;
        double tolerance = ShooterPIDFConfig.tolerance;
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double heading = follower.getPose().getHeading();



        telemetry.addLine("=== TURRET  STATUS ===");
        telemetry.addData("Raw X", x);
        telemetry.addData("Raw Y", y);
        telemetry.addData("Raw Heading", heading);
        telemetry.addData("follower.getPose().getX()", follower.getPose().getX());
        telemetry.addData("follower.getPose().getY()", follower.getPose().getY());
        telemetry.addData("follower.getPose().getHeading()", follower.getPose().getHeading());

//        telemetry.addData("Servo Position", servoPosition);
        telemetry.addData("getCurrentAngleOfTurret()", getCurrentAngleOfTurret());
        telemetry.addData("getPosition()", getPosition());
        telemetry.addData("encoderTurret .getCurrentAngle", robot.axonTurretArmL.getCurrentAngle());
        telemetry.addData("(robot.revEncoderForTurret.getCurrentPosition()/16384) ", (robot.revEncoder.getCurrentPosition()/16384) );
        telemetry.addData("robot.revEncoderForTurret.getCurrentPosition()", (robot.revEncoder.getCurrentPosition()/16384) * (direction.equals(RTPAxon.Direction.FORWARD) ? -360 : 360));



//        telemetry.addData("axonTurretArmL Target Rotation", robot.axonTurretArmL.getTargetRotation());
//        telemetry.addData("encoderTurret .getCurrentPosition()", robot.encoderTurret.getCurrentAngle());

//        telemetry.addData("axonTurretArmL Servo Position", robot.axonTurretArmL.getCurrentAngle());robot.revEncoderForTurret.getCurrentPosition()/16384) * (direction.equals(RTPAxon.Direction.FORWARD) ? -360 : 360)
////        telemetry.addData("axonTurretArmL Total Rotation", robot.axonTurretArmL.getTotalRotation());
//
//        telemetry.addData("encoderTurretArmL", robot.encoderTurretArmL.getVoltage());
//        telemetry.addData("encoderTurretArmR", robot.encoderTurretArmR.getVoltage());
//        telemetry.addData("axonTurretArm getCurrentAngle", robot.axonTurretArmL.getCurrentAngle());

//        telemetry.addData("axonTurretArmL.getPower()", robot.axonTurretArmL.getPower());
//        telemetry.addData("axonTurretArmR.getPower()", robot.axonTurretArmR.getPower());
//
//        telemetry.addData("axonTurretArmR Servo Position", robot.axonTurretArmR.getCurrentAngle());
//        telemetry.addData("axonTurretArmR Total Rotation", robot.axonTurretArmR.getTotalRotation());
//        telemetry.addData("axonTurretArmR Target Rotation", robot.axonTurretArmR.getTargetRotation());
//        telemetry.addData("Servo Position", robot.axonTurretArmR.getCurrentAngle());
//        telemetry.addData("Total Rotation", robot.axonTurretArmR.getTotalRotation());
//        telemetry.addData("Target Rotation", robot.axonTurretArmR.getTargetRotation());

//        telemetry.addLine("=== SHOOTER PIDF TUNING ===");
//        telemetry.addData("Target RPM", "%.0f", ShooterPIDFConfig.targetRPM);
//        telemetry.addData("Current RPM", "%.0f", shooterVelocity);
//        telemetry.addData("currentVelocity", currentVelocity);
//        telemetry.addData("targetVelocity", targetVelocity);
//        double targetVelocity = ShooterPIDFConfig.targetRPM;
//        telemetry.addData("Error", "%.0f RPM", Math.abs(ShooterPIDFConfig.targetRPM - shooterVelocity));
//        telemetry.addData("At Speed?", isShooterAtSpeed ? "YES" : "NO");
//        telemetry.addData("Power", "%.2f", shooterPower);
//        // PIDF 参数值
//        telemetry.addLine("=== PIDF PARAMETERS ===");
//        telemetry.addData("kP", "%.4f", ShooterPIDFConfig.kP);
////        telemetry.addData("kI", "%.4f", ShooterPIDFConfig.kI);
//        telemetry.addData("kD", "%.4f", ShooterPIDFConfig.kD);
//        telemetry.addData("kF", "%.4f", ShooterPIDFConfig.kF);
//        telemetry.addData("Tolerance", "%.0f RPM", ShooterPIDFConfig.tolerance);
//
//        // 从电机状态
//        telemetry.addLine("=== SLAVE MOTOR ===");
//        telemetry.addData("Slave Power", "%.2f", robot.SlaveShooterMotorR.getPower());
//        telemetry.addData("Slave RPM", "%.0f", robot.SlaveShooterMotorR.getVelocity());
//
//        // 射击状态
//        telemetry.addLine("=== SHOOTING STATUS ===");
//        telemetry.addData("Fire Requested", fireRequested ? "YES" : "NO");
//        telemetry.addData("LED Color", isShooterAtSpeed ? "GREEN" : "RED");
    }


    /**
     * 停止射击电机
     */
    private void stopShooter() {
        robot.MasterShooterMotorL.setVelocity(0);
        robot.SlaveShooterMotorR.setPower(0);
        robot.IntakeMotorL.setPower(0);
        robot. IntakeMotorR.setPower(0);
        isShooterAtSpeed = false;
        fireRequested = false;
    }


    ///  ///////////

    private void stopIntake() {
        robot.IntakeMotorL.setPower(intakePowerOff);
        robot.IntakeMotorR.setPower(intakePowerOff);
        robot.MasterShooterMotorL.setPower(ShooterMotorOff);
        robot.SlaveShooterMotorR.setPower(ShooterMotorOff);
        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Non-blocking delay to prevent rapid mode switching
        telemetry.addData("intakePowerOff", intakePowerOff);
        telemetry.update();
        delayTimer.reset();
    }



    /// ///////////

    /**
     * 执行射击序列
     */
    private void executeFireSequence() {
        //       if (!isShooterAtSpeed) return; // 安全检查
        telemetry.addData("Shooter", "FIRING! RPM: %.0f", robot.MasterShooterMotorL.getVelocity());
        telemetry.update();
        // 启动拾取电机将球推入射击器
        robot.IntakeMotorL.setPower(intakePowerShoot);
        robot.IntakeMotorR.setPower(intakePowerShoot); // 全功率推出

        // 保持推球一段时间（根据实际调整）
        ElapsedTime fireTimer = new ElapsedTime();
        while (fireTimer.milliseconds() < 500 && opModeIsActive()) { // 推球500ms
            // 空循环，等待时间到达
            telemetry.addData("Shooter", "Firing... Time: %.0f/500", fireTimer.milliseconds());
            telemetry.update();
        }

        // 停止拾取电机
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        fireRequested = false;

        telemetry.addData("Shooter", "Fire sequence completed");
        telemetry.update();
    }

    /**
     * 更新LED状态显示
     */
    private void updateLEDs() {
        // 检测状态变化，只在变化时更新LED以减少通信负载
//        if (isShooterAtSpeed) {
        if (isShooterAtSpeed) {
            setLEDColor(LED_COLOR_READY);
//            telemetry.addData("LED Status", "READY (GREEN) - Press A to Fire");
        }  else {
            setLEDColor(LED_COLOR_OFF);
//            telemetry.addData("LED Status", "OFF (RED)");
        }
//            wasShooterAtSpeed = false;
//        }
    }

    /**
     * 设置LED颜色（需要根据你的具体LED硬件实现）
     */
    private void setLEDColor(String color) {
        try {
            // 示例：使用REV Blinkin LED驱动
            if (color.equals("GREEN")) {
                robot.greenLED.setState(false);
                robot.redLED.setState(true);
//                robot.greenLED = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            } else if (color.equals("RED")) {
                robot.greenLED.setState(true);
                robot.redLED.setState(false);
            }

        } catch (Exception e) {
            telemetry.addData("LED Error", "Failed to set color: " + color);
        }
    }

    /**
     * 设置射击电机PIDF系数
     */
//    private void setShooterPIDFCoefficients() {
//        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
//        robot.MasterShooterMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//    }

    private double calculateOptimalSlavePower(double masterPower) {

        double masterSpeed = robot.MasterShooterMotorL.getVelocity();
        double slaveSpeed = robot.SlaveShooterMotorR.getVelocity();
        double speedRatio = slaveSpeed / (masterSpeed + 0.001); // 避免除零

        // 如果从电机速度明显落后，增加功率补偿
        if (speedRatio < 0.9) {
            return masterPower * 1.1;
        }
        // 如果从电机速度明显超前，减少功率
        else if (speedRatio > 1.1) {
            return masterPower * 0.9;
        }
        // 正常范围内使用相同功率
        else {
            return masterPower;
        }

    }
    public void updateDrivetrain_FieldCentric() {
        double y = gamepad1.left_stick_y * (1); // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * (1);
        double rx = -gamepad1.right_stick_x * DriveTrains_smoothTurn; //*(0.5) is fine

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
// ******************************************temp
//        if (gamepad1.back) {
//            robot.imu.resetYaw();
//        }
//******************************************temp

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        robot.leftFrontMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
        robot.leftRearMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
        robot.rightFrontMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
        robot.rightRearMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
    }

    public void updateDrivetrain_RobotCentric() {
        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double robot_x = gamepad1.left_stick_x;
        double robot_rx = gamepad1.right_stick_x*DriveTrains_smoothTurn; // If a smooth turn is required 0.5

        double fl = robot_y - robot_x - robot_rx;
        double bl = robot_y + robot_x - robot_rx;
        double fr = robot_y + robot_x + robot_rx;
        double br = robot_y - robot_x + robot_rx;

        robot.leftFrontMotor.setPower(fl * DriveTrains_ReducePOWER);
        robot.leftRearMotor.setPower(bl * DriveTrains_ReducePOWER);
        robot.rightFrontMotor.setPower(fr * DriveTrains_ReducePOWER);
        robot.rightRearMotor.setPower(br * DriveTrains_ReducePOWER);

    }

    private void handleIMUReset() {
        // 检查冷却状态
        if (imuResetInCooldown) {
            if (imuResetTimer.milliseconds() > IMU_RESET_COOLDOWN_MS) {
                imuResetInCooldown = false;
            } else {
                return; // 冷却中，不处理
            }
        }

        // 使用 gamepad1.back 键重设IMU
        if (gamepad1.back) {
            robot.imu.resetYaw();

            // 记录状态和开始冷却
            imuResetInCooldown = true;
            imuResetTimer.reset();

            // 提供视觉反馈
            telemetry.addLine("=== IMU RESET ===");
            telemetry.addData("Status", "Yaw angle reset to 0°");
            telemetry.addData("Current Heading", "%.1f°",
                    robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Cooldown", "%.1fs", IMU_RESET_COOLDOWN_MS / 1000.0);

            // 震动反馈
            gamepad1.rumble(300);
        }
    }
    //    private void handleIMUReset() {
//        // 使用 gamepad1.start 键重设IMU
//        if (gamepad1.start) {
//            robot.imu.resetYaw();
//            telemetry.addData("IMU", "Yaw Axis Reset Complete");
//            telemetry.update();
//            // 短暂震动反馈（如果有gamepad震动功能）
////            if (gamepad1.getGamepadId() == 1) {
////                gamepad1.rumble(300); // 震动300ms
////            }
//            // 防抖延迟
//            sleep(300);
//        }
//    }

//Begin Definition and Initialization of steptestservo()
// Begin debugging with a step increment of 0.05  SGC - servoGamepadControl
//        public void servoGamepadControl() {

/**
 * This code snippet controls the position of a servo motor using the gamepad triggers.
 *
 * **Purpose**:
 * - The left trigger (`gamepad1.left_trigger`) increases the servo's position by a fixed step (`SERVO_STEP`).
 * - The right trigger (`gamepad1.right_trigger`) decreases the servo's position by a fixed step (`SERVO_STEP`).
 * - The servo position is constrained between 0.01 (minimum) and 0.99 (maximum) to prevent invalid values.
 * - The current servo position is displayed on the telemetry for real-time monitoring.
 *
 * **Usage Instructions**:
 * 1. Press the **left trigger** (`gamepad1.left_trigger`) to move the servo incrementally towards its maximum position.
 * 2. Press the **right trigger** (`gamepad1.right_trigger`) to move the servo incrementally towards its minimum position.
 * 3. The servo's position is updated with a small delay (`sleep(200)` milliseconds) to prevent rapid changes from multiple trigger presses.
 * 4. Adjust `SERVO_STEP` as needed to control the increment size for finer or coarser adjustments.
 *
 * **Setup**:
 * - Ensure the servo is connected to the correct port and initialized in the `robot.TServo` variable.
 * - Configure the `SERVO_STEP` variable to determine how much the position changes with each trigger press.
 * - Calibrate the servo movement range (e.g., 0.01 to 0.99) based on your servo's physical limits to avoid damage.
 */


//            if (gamepad1.left_trigger > 0.3) {
//                servoPosition = servoPosition + SERVO_STEP;
//                if (servoPosition >= 1.0) {
//                    servoPosition = 0.99; // 限制最大值
//                }
//                robot.TServo.setPosition(servoPosition);
//                telemetry.addData("Servo Position", servoPosition);
//                telemetry.update();
//                sleep(200);
//            }
//            if (gamepad1.right_trigger > 0.3) {
//                servoPosition = servoPosition - SERVO_STEP;
//                if (servoPosition <= 0.0) {
//                    servoPosition = 0.01; // 限制最小值
//                }
//                robot.TServo.setPosition(servoPosition);
//                telemetry.addData("Servo Position", servoPosition);
//                telemetry.update();
//                sleep(200);
//            }

//End debugging with a step increment of 0.05

//        }
///////////////////End Definition and Initialization of steptestservo()






}








