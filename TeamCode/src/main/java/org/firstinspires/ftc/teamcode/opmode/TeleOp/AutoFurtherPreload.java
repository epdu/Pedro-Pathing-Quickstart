//
///*
//Hello :D my names mikey and i'm the head of software on team 21721. I was looking at the april tag sample code on the PP (pedro pathing) website and it kinda confused me or just wasn't
//what I needed to do, so I decided to make my own! Before you worry about the code itself u need to know a bit about April tags. April tags are basically just QR codes; in the sense
//that when you scan them they give u a numerical value. the april tag values for this season are as the following-
//
//Blue Goal: 20
//Motif GPP: 21
//Motif PGP: 22
//Motif PPG: 23
//Red Goal: 24
//
//So basically, you lineup your robot in front of the motif april tag. It scans said April Tag and then gives you a value back. You then have three if/then statements where you pretty much
//say "if the numeric value is 21, then run the GPP pathbuilder" and so on. Right now, though, the code just has movement. So whenever you get your shooting and intake mechanisms figured out, just add that code in the
//designated function and call the function in whichever part of the pathbuilder it is needed. I hope this helps!
//*/
//
//
////package org.firstinspires.ftc.teamcode.examples;
//package org.firstinspires.ftc.teamcode.pedroPathing;
//// FTC SDK
//
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.Robot;
////import org.firstinspires.ftc.teamcode.lib.pedropathing.Constants;
//import org.firstinspires.ftc.teamcode.util.Alliance;
//
//@Autonomous(name = "PP further zone preload", group = "Opmode")
//@Configurable // Panels
//@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
//public class AutoFurtherPreload extends LinearOpMode {
//    // Initialize elapsed timer
//    private final ElapsedTime runtime = new ElapsedTime();
//    private Timer autoTimer, pathTimer;
//    ShooterSubsystem shooterSubsystem;
//    FlywheelSubsystem flywheelSubsystem;
//    FeederSubsystem feederSubsystem;
//    IntakeSubsystem intakeSubsystem;
//    Vision vision;
//    // Initialize poses
//    //size of robot 16.25x12.5 inch
//    private final Pose startPose = new Pose(92, 6.25, Math.toRadians(0)); // Start Pose further zone of our robot.
//    private final Pose scorePose = new Pose(92, 92.25, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
//    private final Pose scoreEnd = new Pose(92, 92.25, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
//    private final Pose readyPickupPosePPG = new Pose(92, 82.25, Math.toRadians(0)); // PPG  Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose PGPPose = new Pose(92, 60.25, Math.toRadians(0)); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose GPPPose = new Pose(92, 36.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose PPGDONEPose = new Pose(122, 82.25, Math.toRadians(0)); // PPG  Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose PGPDONEPose = new Pose(122, 60.25, Math.toRadians(0)); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose GPPPDONEPose = new Pose(122, 36.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose PARKPose = new Pose(120, 92.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
//
//    // Initialize variables for paths
//
//    private PathChain scorePreload;
//    private PathChain grabPPG;
//    private PathChain scorePPG;
//    private PathChain grabPGP;
//    private PathChain scorePGP;
//    private PathChain grabGPP;
//    private PathChain scoreGPP;
//
//
////    //set April Tag values to specific patterns
////    private static final int PPG_TAG_ID = 23;
////    private static final int PGP_TAG_ID = 22;
////    private static final int GPP_TAG_ID = 21;
////    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
////    private VisionPortal visionPortal;               // Used to manage the video source.
////    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
////    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
//
//
//    // Other variables
//    private Pose currentPose; // Current pose of the robot
//    private Follower follower; // Pedro Pathing follower
//    private TelemetryManager panelsTelemetry; // Panels telemetry
//    private int pathStatePreload;
//    private int pathStatePPG; // Current state machine value
//    private int pathStatePGP; // Current state machine value
//    private int pathStateGPP; // Current state machine value
//
//    public enum PathState {
//        DRIVE_STARTPOS_SHOOTPOS,
//        SHOOT_PRELOAD,
//        DRIVE_SHOOT_READY_PICKUP,
//        END
//    }
//    private Short_3.PathState pathState;
//    private PathChain driveStartShoot, driveReadyPickup;
////    private int foundID; // Current state machine value, dictates which one to run
//
//    public void buildPaths() {
//        driveStartShoot = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, scorePose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                .build();
//        driveReadyPickup = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, readyPickupPosePPG))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), readyPickupPosePPG.getHeading())
//                .build();
//    }
//
//    public void setPathState(Short_3.PathState newState) {
//        pathState = newState;
//        pathTimer.resetTimer();
//    }
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
//
//    @Override
//    public void init() {
////        robot.alliance = Alliance.RED;
//
//        Robot.sendHardwareMap(hardwareMap);
//
//        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
//
//        pathState = Short_3.PathState.DRIVE_STARTPOS_SHOOTPOS;
//
//        pathTimer = new Timer();
//        autoTimer = new Timer();
//        follower = Constants.createFollower(hardwareMap);
//
//        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
//        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
//        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
//        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
//        vision = Vision.getInstance(hardwareMap);
//
//
//
//        intakeSubsystem.init();
//        flywheelSubsystem.init();
//        shooterSubsystem.init();
//        feederSubsystem.init();
//        vision.init();
//
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//
//    @Override
//    public void start() {
//        vision.start();
//
//        autoTimer.resetTimer();
//        setPathState(pathState);
//    }
//
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
//
//    // Custom logging function to support telemetry and Panels
//    private void log(String caption, Object... text) {
//        if (text.length == 1) {
//            telemetry.addData(caption, text[0]);
//            panelsTelemetry.debug(caption + ": " + text[0]);
//        } else if (text.length >= 2) {
//            StringBuilder message = new StringBuilder();
//            for (int i = 0; i < text.length; i++) {
//                message.append(text[i]);
//                if (i < text.length - 1) message.append(" ");
//            }
//            telemetry.addData(caption, message.toString());
//            panelsTelemetry.debug(caption + ": " + message);
//        }
//    }
//
//    // a place to put your intake and shooting functions
//    public void intakeArtifacts() {
//        // Put your intake logic/functions here
//    }
//
//    public void shootArtifacts() {
//        // Put your shooting logic/functions here
//    }
//
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//        initShooterPIDF();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        FtcDashboard Dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = Dashboard.getTelemetry();
//        // 设置 Dashboard 更新频率
//        Dashboard.setTelemetryTransmissionInterval(100); // 100ms 更新一次
//
//
//        // Initialize Panels telemetry
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        // Initialize Pedro Pathing follower
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//
//        // Log completed initialization to Panels and driver station (custom log function)
//        log("Status", "Initialized");
//        telemetry.update(); // Update driver station after logging
//
//        // Wait for the game to start (driver presses START)
//        waitForStart();
//        runtime.reset();
//
//        setpathStatePreload(0);
//        setpathStatePPG(0);
//        setpathStatePGP(0);
//        setpathStateGPP(0);
//        runtime.reset();
//
//        while (opModeIsActive()) {
//            // Update Pedro Pathing and Panels every iteration
//            follower.update();
//            panelsTelemetry.update();
//            currentPose = follower.getPose(); // Update the current pose
//
//
//            buildPathsPreload();
//            updateStateMachinePreload();
//            buildPathsPPG();
//            updateStateMachinePPG();
////            buildPathsPGP();
////            updateStateMachinePGP();
////            buildPathsGPP();
////            updateStateMachineGPP();
//
//
//            // Log to Panels and driver station (custom log function)
//            log("Elapsed", runtime.toString());
//            log("X", currentPose.getX());
//            log("Y", currentPose.getY());
//            log("Heading", currentPose.getHeading());
//            telemetry.update(); // Update the driver station after logging
//        }
//    }
//
//    public void buildPathsPreload(){
//        scorePreload = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, scorePose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                .build();
//
//    }
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
//
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
//
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
//
//    //below is the state machine or each pattern
//    public void updateStateMachinePreload() {
//        switch (pathStatePreload) {
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
//                    follower.followPath(scorePreload);
//                    setpathStatePPG(-1); //set it to -1 so it stops the state machine execution
//                }
//                break;
//        }
//    }
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
//
//
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
//
//
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
//
//    // Setter methods for pathState variables placed at the class level
//    void setpathStatePreload(int newPathState) {
//        this.pathStatePreload = newPathState;
//    }
//    void setpathStatePPG(int newPathState) {
//        this.pathStatePPG = newPathState;
//    }
//
//    void setpathStatePGP(int newPathState) {
//        this.pathStatePGP = newPathState;
//    }
//
//    void setpathStateGPP(int newPathState) {
//        this.pathStateGPP = newPathState;
//    }
//
//
//
//}