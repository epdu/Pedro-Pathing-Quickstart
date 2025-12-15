
/*
Hello :D my names mikey and i'm the head of software on team 21721. I was looking at the april tag sample code on the PP (pedro pathing) website and it kinda confused me or just wasn't
what I needed to do, so I decided to make my own! Before you worry about the code itself u need to know a bit about April tags. April tags are basically just QR codes; in the sense
that when you scan them they give u a numerical value. the april tag values for this season are as the following-

Blue Goal: 20
Motif GPP: 21
Motif PGP: 22
Motif PPG: 23
Red Goal: 24

So basically, you lineup your robot in front of the motif april tag. It scans said April Tag and then gives you a value back. You then have three if/then statements where you pretty much
say "if the numeric value is 21, then run the GPP pathbuilder" and so on. Right now, though, the code just has movement. So whenever you get your shooting and intake mechanisms figured out, just add that code in the
designated function and call the function in whichever part of the pathbuilder it is needed. I hope this helps!
*/


//package org.firstinspires.ftc.teamcode.examples;
package org.firstinspires.ftc.teamcode.pedroPathing;
// FTC SDK

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Short_6;
//import org.firstinspires.ftc.teamcode.auto.red.Short_6;
//import org.firstinspires.ftc.teamcode.lib.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

@Autonomous(name = "AutoPreload9212 one preload", group = "Opmode")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class AutoPreload9212 extends LinearOpMode {
    HardwareQualifier robot = new HardwareQualifier();
    private volatile boolean isRunning = true;
    ElapsedTime delayTimer = new ElapsedTime();
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();
    private Timer autoTimer, pathTimer;
//    ShooterSubsystem shooterSubsystem;
//    FlywheelSubsystem flywheelSubsystem;
//    FeederSubsystem feederSubsystem;
//    IntakeSubsystem intakeSubsystem;
    Vision vision;
    private static final double VELOCITY_TOLERANCE = 30; // RPM容差，可根据测试调整
    // 状态变量
    private boolean isShooterAtSpeed = false;
    private boolean wasShooterAtSpeed = false; // 用于检测状态变化
    private boolean fireRequested = false;
    // LED颜色常量（根据你的LED库调整）
    private final String LED_COLOR_READY = "GREEN";
    private final String LED_COLOR_ACCELERATING = "YELLOW";
    private final String LED_COLOR_OFF = "RED";

    // RPM = (TPS * 60秒) / 每转ticks数
//    return (tps * 60.0) / ticksPerRevolution;  28*13.7
    private static final double Close_SHOOTER_TARGET_RPM = 800;//  400RPM---2,557.33333333333333
    private static final double Med_SHOOTER_TARGET_RPM = 1300;   //1598 white tri a little bit too far//  250RPM---1586.67
    private static final double Far_SHOOTER_TARGET_RPM = 2237;  //  350RPM---2237
    //  1000RPM---6346.67
    //  600RPM---3808
    //  500RPM---3173.3

    public float DriveTrains_ReducePOWER=0.75f;
    public float DriveTrains_smoothTurn=0.55f;
    public String fieldOrRobotCentric = "robot";
    //    public String fieldOrRobotCentric = "field";
    private double powerMultiplier = 0.9;
    boolean move = false;
    int controlMode = 1;
    public float  intakePowerIntake=0.85f;
    public float  intakePowerShoot=0.9f;
    public float  intakePowerDump=-0.6f;
    public float  intakePowerOff=0.0f;
    public float  ShooterMotorShootFar=0.95f;
    public float  ShooterMotorShootMed=-0.8f;
    public float  ShooterMotorShootClose=-0.8f;
    public float  ShooterMotorHold=-0.2f;
    public float  ShooterMotorClean=-0.8f;
    public float  ShooterMotorOff=0.0f;
    public static final double HoodArmPositionInit = 0.1;
    public static final double HoodArmPositionCloseShoot = 0.3;
    public static final double HoodArmPositionMedShoot = 0.2;
    // Initialize poses
    //size of robot 16.25x12.5 inch
    private final Pose startPose = new Pose(92, 6.25, Math.toRadians(0)); // Start Pose further zone of our robot.
    private final Pose scorePose = new Pose(92, 92.25, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose scoreEnd = new Pose(92, 92.25, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose readyPickupPosePPG = new Pose(92, 82.25, Math.toRadians(0)); // PPG  Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PGPPose = new Pose(92, 60.25, Math.toRadians(0)); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPose = new Pose(92, 36.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose PPGDONEPose = new Pose(122, 82.25, Math.toRadians(0)); // PPG  Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PGPDONEPose = new Pose(122, 60.25, Math.toRadians(0)); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPDONEPose = new Pose(122, 36.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose PARKPose = new Pose(120, 92.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose offlinePose = new Pose(120, 92.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    // Initialize variables for paths
    private PathChain scorePreload;
    private PathChain grabPPG;
    private PathChain scorePPG;
    private PathChain grabPGP;
    private PathChain scorePGP;
    private PathChain grabGPP;
    private PathChain scoreGPP;


//    //set April Tag values to specific patterns
//    private static final int PPG_TAG_ID = 23;
//    private static final int PGP_TAG_ID = 22;
//    private static final int GPP_TAG_ID = 21;
//    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
//    private VisionPortal visionPortal;               // Used to manage the video source.
//    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
//    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    // Other variables
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathStatePreload;
    private int pathStatePPG; // Current state machine value
    private int pathStatePGP; // Current state machine value
    private int pathStateGPP; // Current state machine value

    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        DRIVE_START_POS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_READY_PICKUP_POS,
        PICKUP,
        DRIVE_BACK_SHOOT_POS,
        SHOOT_PICKUP,
        DRIVE_OFFLINE,
//        DRIVE_OFFLINE,
        END
    }
    private Short_6.PathState pathState;
    private PathChain driveStartShoot;
    private PathChain driveReadyPickup;
    private PathChain driveOffline;
//    private int foundID; // Current state machine value, dictates which one to run

    public void buildPaths() {
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        driveReadyPickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, readyPickupPosePPG))
                .setLinearHeadingInterpolation(scorePose.getHeading(), readyPickupPosePPG.getHeading())
                .build();
        driveOffline = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, offlinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), offlinePose.getHeading())
                .build();
    }

    public void setPathState(Short_6.PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
//    public void statePathUpdate() {
//        switch (pathState) {
//            case DRIVE_STARTPOS_SHOOTPOS:
//                follower.followPath(driveStartShoot, false);
//                setPathState(Short_3.PathState.SHOOT_PRELOAD);
//                break;
//            case SHOOT_PRELOAD:
//                if (!follower.isBusy()) {
//                    shooterSubsystem.shoot(false);
//                    feederSubsystem.autoFeed();
//                }
//
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 8) {
//                    follower.followPath(driveReadyPickup);
//                    setPathState(Short_3.PathState.DRIVE_SHOOT_READY_PICKUP);
//                }
//                break;
//            case DRIVE_SHOOT_READY_PICKUP:
//                if (!follower.isBusy()) {
//                    setPathState(Short_3.PathState.END);
//                }
//            case END:
//                flywheelSubsystem.stop();
//                shooterSubsystem.setAngle(0);
//                feederSubsystem.stop();
//            default:
//                break;
//        }
//    }





//    @Override
//    public void loop() {
//        vision.loop();
//
//        follower.update();
//        statePathUpdate();
//
//        telemetry.addData("State", pathState.toString());
//        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
//        telemetry.addLine();
//
//
//        telemetry.addData("Target Angle", shooterSubsystem.targetPos);
//        telemetry.addData("Current Angle", shooterSubsystem.getPosition());
//        telemetry.addLine();
//
//        telemetry.addData("Flywheel Velocity", flywheelSubsystem.getVelocity());
//        telemetry.addData("Flywheel Target", flywheelSubsystem.lastTargetRadPerSec);
//        telemetry.addData("Flywheel Volts", flywheelSubsystem.lastTargetVolts);
//        telemetry.addLine();
//
//
//
//        telemetry.addLine("//Vision//");
//        telemetry.addData("LL Valid", vision.llValid);
//        telemetry.addData("Has Tag", vision.hasTag);
//        telemetry.addData("Ta", vision.getTa().orElse(-1.0));
//        telemetry.addData("Tx", vision.getTx().orElse(-1.0));
//        telemetry.addData("Ty", vision.getTy().orElse(-1.0));
//        telemetry.addData("Distance", vision.getDistance().orElse(-1.0));
//        telemetry.addLine();
//    }

    // Custom logging function to support telemetry and Panels
    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }

    // a place to put your intake and shooting functions
    public void intakeArtifacts() {
        // Put your intake logic/functions here
    }

    public void shootArtifacts() {
        // Put your shooting logic/functions here
    }


    @Override
    public void runOpMode() {

        /// ///////////////////////////

            Robot.sendHardwareMap(hardwareMap);

            telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

            pathState = Short_6.PathState.DRIVE_START_POS_SHOOT_POS;

            pathTimer = new Timer();
            autoTimer = new Timer();
            follower = Constants.createFollower(hardwareMap);

//            intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
//            flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
//            shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
//            feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
            vision = Vision.getInstance(hardwareMap);

//            intakeSubsystem.init();
//            flywheelSubsystem.init();
//            shooterSubsystem.init();
//            feederSubsystem.init();
            vision.init();

            buildPaths();
            follower.setStartingPose(startPose);




        /// //////////////////////////

        robot.init(hardwareMap);
        initShooterPIDF();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard Dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = Dashboard.getTelemetry();
        // 设置 Dashboard 更新频率
        Dashboard.setTelemetryTransmissionInterval(100); // 100ms 更新一次

/// //////////////
//            vision.start();
            autoTimer.resetTimer();
            setPathState(pathState);
/// //////////////////

        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        setpathStatePreload(0);
//        setpathStatePPG(0);
//        setpathStatePGP(0);
//        setpathStateGPP(0);
        runtime.reset();

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Update the current pose

            statePathUpdate();

            buildPathsPreload();
            updateStateMachinePreload();
//            buildPathsPPG();
//            updateStateMachinePPG();
//            buildPathsPGP();
//            updateStateMachinePGP();
//            buildPathsGPP();
//            updateStateMachineGPP();


            // Log to Panels and driver station (custom log function)
            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            telemetry.update(); // Update the driver station after logging
        }
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.followPath(driveStartShoot, true);
                setPathState(Short_6.PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    startShooter();
                    delayTimer.reset();
                    while (delayTimer.milliseconds() < 1000 && opModeIsActive()) {
                        // Other tasks can be processed here
                    } //
                    executeFireSequence();
                    robot.IntakeMotor.setPower(intakePowerShoot);
//                    shooterSubsystem.shoot(false);
//                    feederSubsystem.autoFeed();
//                    intakeSubsystem.intake();
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    stopShooter() ;

//                    flywheelSubsystem.stop();
//                    feederSubsystem.stop();
//                    intakeSubsystem.stop();

                    follower.followPath(driveReadyPickup);
                    setPathState(Short_6.PathState.DRIVE_OFFLINE);
                }
                break;
//            case DRIVE_READY_PICKUP_POS:
//                if (!follower.isBusy()) {
//                    follower.followPath(drivePickup, .5, true);
//                    setPathState(Short_6.PathState.PICKUP);
//                }
//                break;
//            case PICKUP:
//                if (pathTimer.getElapsedTimeSeconds() < 1.75) {
//                    intakeSubsystem.intake();
//                    feederSubsystem.feed();
//                    flywheelSubsystem.setPower(1);
//                }
//
//                if (pathTimer.getElapsedTimeSeconds() > 1.75 && pathTimer.getElapsedTimeSeconds() < 2) {
//                    intakeSubsystem.stop();
//                    feederSubsystem.stop();
//                    flywheelSubsystem.stop();
//                }
//
//
//                if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() < 2.5) {
//                    intakeSubsystem.intake();
//                    flywheelSubsystem.setPower(1);
//                    feederSubsystem.back();
//                }
//
//                if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 2.75) {
//                    intakeSubsystem.intake();
//                    flywheelSubsystem.setPower(1);
//                    feederSubsystem.feed();
//                }
//
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
//                    follower.followPath(drivePickupShoot);
//                    setPathState(Short_6.PathState.DRIVE_BACK_SHOOT_POS);
//                }
//                break;
//            case DRIVE_BACK_SHOOT_POS:
//                intakeSubsystem.stop();
//                feederSubsystem.stop();
//                flywheelSubsystem.stop();
//
//
//
//                if (!follower.isBusy()) {
//                    setPathState(Short_6.PathState.SHOOT_PICKUP);
//                }
//                break;
//
//            case SHOOT_PICKUP:
//                if (!follower.isBusy()) {
//                    shooterSubsystem.shoot(false);
//                    feederSubsystem.autoFeed();
//                    intakeSubsystem.intake();
//                }
//
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
//                    follower.followPath(driveOffline);
//                    setPathState(Short_6.PathState.DRIVE_OFFLINE);
//                }
//                break;
            case DRIVE_OFFLINE:
                if (!follower.isBusy()) {
                    setPathState(Short_6.PathState.END);
                }
                break;
            case END:
                terminateOpModeNow();
                break;
            default:
                break;
        }
    }




    public void buildPathsPreload(){
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

    }
//    public void buildPathsPPG() {
//        // basically just plotting the points for the lines that score the PPG pattern
//
//
//        grabPPG = follower.pathBuilder() //
//                .addPath(new BezierLine(startPose, PPGPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), PPGPose.getHeading())
//                .build();
//
//        // Move to the scoring pose from the first artifact pickup pose
//        scorePPG = follower.pathBuilder()
//                .addPath(new BezierLine(PPGPose, scorePose))
//                .setLinearHeadingInterpolation(PPGPose.getHeading(), scorePose.getHeading())
//                .build();
//    }

//    public void buildPathsPGP() {
//        // basically just plotting the points for the lines that score the PGP pattern
//
//        // Move to the first artifact pickup pose from the start pose
//        grabPGP = follower.pathBuilder() // Changed from scorePGP to grabPGP
//                .addPath(new BezierLine(startPose, PGPPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), PGPPose.getHeading())
//                .build();
//
//        // Move to the scoring pose from the first artifact pickup pose
//        scorePGP = follower.pathBuilder()
//                .addPath(new BezierLine(PGPPose, scorePose))
//                .setLinearHeadingInterpolation(PGPPose.getHeading(), scorePose.getHeading())
//                .build();
//    }

//    public void buildPathsGPP() {
//        // basically just plotting the points for the lines that score the GPP pattern
//
//        // Move to the first artifact pickup pose from the start pose
//        grabGPP = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, GPPPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), GPPPose.getHeading())
//                .build();
//
//        // Move to the scoring pose from the first artifact pickup pose
//        scoreGPP = follower.pathBuilder()
//                .addPath(new BezierLine(GPPPose, scorePose))
//                .setLinearHeadingInterpolation(GPPPose.getHeading(), scorePose.getHeading())
//                .build();
//    }

    //below is the state machine or each pattern
    public void updateStateMachinePreload() {
        switch (pathStatePreload) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(scorePreload);
                setpathStatePreload(1); // Call the setter method
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Move to the first artifact pickup location from the scoring position
                    follower.followPath(scorePreload);
                    setpathStatePreload(-1); //set it to -1 so it stops the state machine execution
                }
                break;
        }
    }
//    public void updateStateMachinePPG() {
//        switch (pathStatePPG) {
//            case 0:
//                // Move to the scoring position from the start position
//                follower.followPath(grabPPG);
//                setpathStatePPG(1); // Call the setter method
//                break;
//            case 1:
//                // Wait until we have passed all path constraints
//                if (!follower.isBusy()) {
//
//                    // Move to the first artifact pickup location from the scoring position
//                    follower.followPath(scorePPG);
//                    setpathStatePPG(-1); //set it to -1 so it stops the state machine execution
//                }
//                break;
//        }
//    }


//    public void updateStateMachinePGP() {
//        switch (pathStatePGP) {
//            case 0:
//                // Move to the scoring position from the start position
//                follower.followPath(grabPGP);
//                setpathStatePGP(1); // Call the setter method
//                break;
//            case 1:
//                // Wait until we have passed all path constraints
//                if (!follower.isBusy()) {
//
//                    // Move to the first artifact pickup location from the scoring position
//                    follower.followPath(scorePGP);
//                    setpathStatePGP(-1); // Call the setter for PGP
//                }
//                break;
//        }
//    }


//    public void updateStateMachineGPP() {
//        switch (pathStateGPP) {
//            case 0:
//                // Move to the scoring position from the start position
//                follower.followPath(grabGPP);
//                setpathStateGPP(1); // Call the setter method
//                break;
//            case 1:
//                // Wait until we have passed all path constraints
//                if (!follower.isBusy()) {
//
//                    // Move to the first artifact pickup location from the scoring position
//                    follower.followPath(scoreGPP);
//                    setpathStateGPP(-1); //set it to -1 so it stops the state machine execution
//                }
//                break;
//        }
//    }
public void updateIntake() {
    // 手柄控制拾取电机
    if (gamepad1.left_trigger > 0.1) {
        // 吸入
        robot.IntakeMotor.setPower(intakePowerIntake);

        robot.MasterShooterMotorL.setPower(ShooterMotorHold);
        robot.SlaveShooterMotorR.setPower(ShooterMotorHold);
        // Non-blocking delay to prevent rapid mode switching

        telemetry.addData("intakePowerIntake", intakePowerIntake);
        telemetry.update();
        delayTimer.reset();
        while (delayTimer.milliseconds() < 300 && opModeIsActive()) {
            // Other tasks can be processed here
        } // 防止快速连击导致模式快速切换

    } else if (gamepad1.left_bumper) {
        // 反转（吐出）
        robot.IntakeMotor.setPower(intakePowerDump);

        telemetry.addData("intakePowerDump", intakePowerDump);
        telemetry.update();
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

        // 手柄控制发射电机 - 按下右肩键启动射击电机
        if (gamepad1.right_trigger > 0.1) {
            // 检查射击电机是否达到目标速度
            checkShooterVelocity();
            if (!robot.MasterShooterMotorL.isBusy()){
                startShooter();
            }

        }
        // 手动射击触发 - 按下A键射击（仅在速度达标时有效）
//        if (gamepad1.right_bumper && isShooterAtSpeed && !fireRequested)
        if (gamepad1.right_bumper) {
            fireRequested = true;
            executeFireSequence();
            robot.IntakeMotor.setPower(intakePowerShoot);
        }
        // 停止射击电机 - 按下B键
        if (gamepad1.b) {
            stopShooter();
            robot.IntakeMotor.setPower(intakePowerOff);
            fireRequested = false;
        }

        // 如果射击电机未运行，确保重置状态
        if (!gamepad1.right_bumper && !robot.MasterShooterMotorL.isBusy()) {
            isShooterAtSpeed = false;
            fireRequested = false;
        }
        // 更新射击系统遥测数据（关键！）
        updateShooterTelemetry();
    }

    public static class ShooterPIDFConfig {
        public static double kP = 100;     // 比例增益0.10.350.651.0655
        public static double kI = 0.0;      // 积分增益
        public static double kD = 0.0;      // 微分增益
        public static double kF = 17;      // 前馈增益0.050.08
        //        P: 20–60
//        I: 0–0.002
//        D: 0–0.5
//        F: 10–30
//        public static double targetRPM =Close_SHOOTER_TARGET_RPM;
        public static double targetRPM =Med_SHOOTER_TARGET_RPM; // 目标转速
        public static double tolerance = 10;    // 转速容差
    }

    private void initShooterPIDF() {
        // 初始化时调用一次
        if (robot.MasterShooterMotorL instanceof DcMotorEx) {
            DcMotorEx shooter = (DcMotorEx) robot.MasterShooterMotorL;
            // 设置运行模式
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // 设置PIDF参数
            PIDFCoefficients pidf = new PIDFCoefficients(
                    TeleOpQualifier.ShooterPIDFConfig.kP,
                    TeleOpQualifier.ShooterPIDFConfig.kI,
                    TeleOpQualifier.ShooterPIDFConfig.kD,
                    TeleOpQualifier.ShooterPIDFConfig.kF
            );
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            telemetry.addData("Shooter PIDF", "Initialized");
        }
    }

    private void startShooter() {
        robot.IntakeMotor.setPower(0);
        if (robot.MasterShooterMotorL instanceof DcMotorEx) {
            DcMotorEx shooter = (DcMotorEx) robot.MasterShooterMotorL;
            // 直接使用setVelocity，它会使用已配置的PIDF
            shooter.setVelocity(TeleOpQualifier.ShooterPIDFConfig.targetRPM);

            // 从电机使用简单功率跟随（可选PIDF）
//            robot.SlaveShooterMotorR.setPower(shooter.getPower() * 0.95); // 95%跟随
//
            double MasterShooterMotorLPower = robot.MasterShooterMotorL.getPower();
            robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double SlaveShooterMotorRPower = calculateOptimalSlavePower(MasterShooterMotorLPower);
            robot.SlaveShooterMotorR.setPower(SlaveShooterMotorRPower);

        }

    }



/// //stop////////
///
///
///
///
///

    /**
     * 检查射击电机速度（使用 Dashboard 调整的容差）
     */
    private void checkShooterVelocity() {
        if (robot.MasterShooterMotorL.isBusy()) {
            double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());
            double targetVelocity = TeleOpQualifier.ShooterPIDFConfig.targetRPM;
            double tolerance = TeleOpQualifier.ShooterPIDFConfig.tolerance;

            // 检查是否在容差范围内
            if (Math.abs(currentVelocity - targetVelocity) <= tolerance) {
                isShooterAtSpeed = true;
            } else {
                isShooterAtSpeed = false;
                fireRequested = false;
            }
        } else {
            isShooterAtSpeed = false;
        }
    }


    /**
     * 更新射击系统遥测数据（用于 PIDF 调参）
     */
    private void updateShooterTelemetry() {
        // 射击电机状态
        double shooterVelocity = robot.MasterShooterMotorL.getVelocity();
        double shooterPower = robot.MasterShooterMotorL.getPower();
        double shooterCurrent = robot.MasterShooterMotorL.getCurrent(CurrentUnit.AMPS); // 如果有电流传感器

        telemetry.addLine("=== SHOOTER PIDF TUNING ===");
        telemetry.addData("Target RPM", "%.0f", TeleOpQualifier.ShooterPIDFConfig.targetRPM);
        telemetry.addData("Current RPM", "%.0f", shooterVelocity);
        telemetry.addData("Error", "%.0f RPM", Math.abs(TeleOpQualifier.ShooterPIDFConfig.targetRPM - shooterVelocity));
        telemetry.addData("At Speed?", isShooterAtSpeed ? "YES" : "NO");
        telemetry.addData("Power", "%.2f", shooterPower);

        // PIDF 参数值
        telemetry.addLine("=== PIDF PARAMETERS ===");
        telemetry.addData("kP", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kP);
        telemetry.addData("kI", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kI);
        telemetry.addData("kD", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kD);
        telemetry.addData("kF", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kF);
        telemetry.addData("Tolerance", "%.0f RPM", TeleOpQualifier.ShooterPIDFConfig.tolerance);

        // 从电机状态
        telemetry.addLine("=== SLAVE MOTOR ===");
        telemetry.addData("Slave Power", "%.2f", robot.SlaveShooterMotorR.getPower());
        telemetry.addData("Slave RPM", "%.0f", robot.SlaveShooterMotorR.getVelocity());

        // 射击状态
        telemetry.addLine("=== SHOOTING STATUS ===");
        telemetry.addData("Fire Requested", fireRequested ? "YES" : "NO");
        telemetry.addData("LED Color", isShooterAtSpeed ? "GREEN" : "RED");
    }


    /**
     * 停止射击电机
     */
    private void stopShooter() {
        robot.MasterShooterMotorL.setVelocity(0);
        robot.SlaveShooterMotorR.setPower(0);
        robot.IntakeMotor.setPower(0);
        isShooterAtSpeed = false;
        fireRequested = false;
    }


    ///  ///////////

    private void stopIntake() {
        robot.IntakeMotor.setPower(intakePowerOff);
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
        robot.IntakeMotor.setPower(intakePowerShoot); // 全功率推出

        // 保持推球一段时间（根据实际调整）
        ElapsedTime fireTimer = new ElapsedTime();
        while (fireTimer.milliseconds() < 500 && opModeIsActive()) { // 推球500ms
            // 空循环，等待时间到达
            telemetry.addData("Shooter", "Firing... Time: %.0f/500", fireTimer.milliseconds());
            telemetry.update();
        }

        // 停止拾取电机
        robot.IntakeMotor.setPower(0);
        fireRequested = false;

        telemetry.addData("Shooter", "Fire sequence completed");
        telemetry.update();
    }

    /**
     * 更新LED状态显示
     */
    private void updateLEDs() {
        // 检测状态变化，只在变化时更新LED以减少通信负载
        if (isShooterAtSpeed != wasShooterAtSpeed) {
            if (isShooterAtSpeed) {
                setLEDColor(LED_COLOR_READY);
                telemetry.addData("LED Status", "READY (GREEN) - Press A to Fire");
            }  else {
                setLEDColor(LED_COLOR_OFF);
                telemetry.addData("LED Status", "OFF (RED)");
            }
            wasShooterAtSpeed = isShooterAtSpeed;
        }
    }

    /**
     * 设置LED颜色（需要根据你的具体LED硬件实现）
     */
    private void setLEDColor(String color) {
        try {
            // 示例：使用REV Blinkin LED驱动
            if (color.equals("GREEN")) {
                robot.greenLED.setState(true);
                robot.redLED.setState(false);
//                robot.greenLED = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            } else if (color.equals("RED")) {
                robot.greenLED.setState(false);
                robot.redLED.setState(true);
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






    // Setter methods for pathState variables placed at the class level
    void setpathStatePreload(int newPathState) {
        this.pathStatePreload = newPathState;
    }
    void setpathStatePPG(int newPathState) {
        this.pathStatePPG = newPathState;
    }

    void setpathStatePGP(int newPathState) {
        this.pathStatePGP = newPathState;
    }

    void setpathStateGPP(int newPathState) {
        this.pathStateGPP = newPathState;
    }



}