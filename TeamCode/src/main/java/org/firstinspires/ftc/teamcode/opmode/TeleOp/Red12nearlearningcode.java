////package org.firstinspires.ftc.teamcode.pedroPathing;
////
////import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpQualifier.blockageblockposition;
////
////import com.acmerobotics.dashboard.FtcDashboard;
////import com.acmerobotics.dashboard.config.Config;
////import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
////import com.bylazar.telemetry.TelemetryManager;
////import com.pedropathing.follower.Follower;
////import com.pedropathing.geometry.BezierCurve;
////import com.pedropathing.geometry.BezierLine;
////import com.pedropathing.geometry.Pose;
////import com.pedropathing.paths.PathChain;
////import com.pedropathing.util.Timer;
////import com.qualcomm.hardware.limelightvision.Limelight3A;
////import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.DcMotorEx;
////import com.qualcomm.robotcore.hardware.PIDFCoefficients;
////import com.qualcomm.robotcore.util.ElapsedTime;
////
////import org.firstinspires.ftc.robotcore.external.Telemetry;
////
////// 姿态
////// Limelight
////// 位姿
////@Autonomous(name = "RED Near opengate NO intake TWELVE  park after third loaded")
////// intake second role with open gate , park at  136, 36.25 with three loaded
//////. 3 ARTIFACTS on each SPIKE MARK arranged as follows:
//////i.Near (audience side): GPP
//////ii. Middle: PGP
//////iii. Far (GOAL side): PPG
////public class Red12nearlearningcode extends LinearOpMode {
////    HardwareQualifier robot = new HardwareQualifier();
////   private Limelight3A limelight;
////    private volatile boolean isRunning = true;
////    private boolean shooterStarted=false;
////    // Initialize elapsed timer
////    private final ElapsedTime runtime = new ElapsedTime();
////    private Timer autoTimer, pathTimer,shootTimer;
////    private static final double VELOCITY_TOLERANCE = 30; // RPM容差，可根据测试调整
////    // 状态变量
////    private boolean isShooterAtSpeed = false;
////    private boolean wasShooterAtSpeed = false; // 用于检测状态变化
////    private boolean firstPickupCompleted = false;
////    private boolean fireRequested = false;
////    // LED颜色常量（根据你的LED库调整）
////    private final String LED_COLOR_READY = "GREEN";
////    private final String LED_COLOR_ACCELERATING = "YELLOW";
////    private final String LED_COLOR_OFF = "RED";
//////  RPM = (TPS * 60秒) / 每转ticks数
//////  return (tps * 60.0) / ticksPerRevolution;  28*13.7     28*13.7-----28 6000RPM
//////  TPS=RPM/60*ticksPerRevolution=RPM*28*13.7/60；
////    private static final double Close_SHOOTER_TARGET_RPM = 100;//  400RPM---2,557.33333333333333
//////    private static final double Med_SHOOTER_TARGET_RPM = 204;   //1598 white tri a little bit too far//  250RPM---1586.67
////    private static final double Med_SHOOTER_TARGET_RPM = 2785;   //1598 white tri a little bit too far//  250RPM---1586.67//150-100 too big
//////    private static final double Med_SHOOTER_TARGET_Velocity = 1300;
////    private static final double Med_SHOOTER_TARGET_Velocity = 1200; //1598 white tri a little bit too far//  250RPM---1586.67//150-100 too big
////    private static final double Far_SHOOTER_TARGET_RPM = 350;  //  350RPM---2237
//////   private static final double Close_SHOOTER_TARGET_RPM = 800;//  400RPM---2,557.33333333333333
//////    private static final double Med_SHOOTER_TARGET_RPM = 1300;   //1598 white tri a little bit too far//  250RPM---1586.67
//////    private static final double Far_SHOOTER_TARGET_RPM = 2237;  //  350RPM---2237
////    public float DriveTrains_ReducePOWER=0.75f;
////    public float DriveTrains_smoothTurn=0.55f;
//////    public String fieldOrRobotCentric = "robot";
////    public String fieldOrRobotCentric = "field";
////    private double powerMultiplier = 0.9;
////    boolean move = false;
////    int controlMode = 1;
////    public float  intakePowerIntake=0.75f;//0.95
////    public float  intakePowerShoot=0.85f;//0.9
//////    public float  intakePowerShoot=0.8f;//0.9
////    public float  intakePowerDump=-0.6f;
////    public float  intakePowerOff=0.0f;
////    public float  ShooterMotorShootFar=0.95f;
////    public float  ShooterMotorShootMed=-0.8f;
////    public float  ShooterMotorShootClose=-0.8f;
////    public float  ShooterMotorHold=-0.2f;
////    public float  ShooterMotorClean=-0.8f;
////    public float  ShooterMotorOff=0.0f;
////    public static final double HoodArmPositionInit = 0.1;
////    public static final double HoodArmPositionCloseShoot = 0.3;
////    public static final double HoodArmPositionMedShoot = 0.2;
////    // Other variables
////    private Pose currentPose; // Current pose of the robot
////    private Follower follower; // Pedro Pathing follower
////    private TelemetryManager panelsTelemetry; // Panels telemetry
////    private int pathStatePreload;
////    private int pathStatePPG; // Current state machine value
////    private int pathStatePGP; // Current state machine value
////    private int pathStateGPP; // Current state machine value
////    private boolean hasPathStarted = false;
////    private PathState pathState;
////    private AutoShootState autoShootState;
//////    ShooterSubsystem shooterSubsystem;
//////    FlywheelSubsystem flywheelSubsystem;
//////    FeederSubsystem feederSubsystem;
//////    IntakeSubsystem intakeSubsystem;
////    private PathChain driveStartShoot;
////    private PathChain driveReadyThirdPickup, driveThirdPickup, driveThirdPickupShoot;
////    private PathChain driveReadyOpenGatePickup, driveOpenGatePickup, driveOpenGatePickupShoot;
////    private PathChain driveReadySecondPickup,driveSecondPickup, driveSecondPickupShoot;
////    private PathChain driveReadyFirstPickup, driveFirstPickup,driveFirstPickupShoot;
////    private PathChain driveOffline,driveOfflineofFirst;
////    public enum PathState {
////        DRIVE_START_POS_SHOOT_POS,
////        DRIVE_TO_SHOOT_WAIT,
////        SHOOT_PRELOAD,
////        DRIVE_READY_THIRD_PICKUPT_POS,
////        THIRD_PICKUP,
////        //
////         //
////         //
////
////        SHOOT_FIRST_PICKUP,
////        DRIVE_OFFLINE,
////        END
////    }
////    private final Pose startPose = new Pose(127.5,118.75, Math.toRadians(0)); // Start Pose further zone of our robot.
////    private final Pose shootPose = new Pose(92, 92.25, Math.toRadians(45));
////    private final Pose scoreEnd = new Pose(92, 92.25, Math.toRadians(0)); //
////    private final Pose readyThirdPickupPose = new Pose();
////    private final Pose readySecondPickupPose = new Pose();
////    private final Pose readyFirstPickupPose = new Pose();
////    private final Pose readyOpenGatePickupPose = new Pose(); //
////    private final Pose thirdPickupPose = new Pose(); // .
////    private final Pose secondPickupPose = new Pose();
////    private final Pose secondPickupCP = new Pose();
////    private final Pose openGatePickupCP = new Pose();
////    private final Pose secondPickupPoseControlPoint = new Pose(); //
////    private final Pose firstPickupPose = new Pose(); // .
////    private final Pose openGatePickupPose = new Pose();
////
////    private final Pose offlinePose = new Pose(112, 92.25, Math.toRadians(0)); //
////    @Override
////    public void runOpMode() {
////
////        /* ---------------- INIT ---------------- */
//////        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
///////////////////////////////////////////////////////////
//////
//////        Robot.alliance = Alliance.RED;
//////        Robot.sendHardwareMap(hardwareMap);
////
////        robot.init(hardwareMap);
//////        PanelsTelemetry.INSTANCE.init(hardwareMap);
//////        robot.imu.resetYaw();
////        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////        robot.BlockageArm.setPosition(blockageblockposition);////switch with blockage case with blockage
////        robot.BlockageArm.setPosition(blockageblockposition); //switch with blockage case with blockage need command
//////        follower.setStartingPose(startPose);
////
////
/////// ///////////////////////////////////////////////debug PIDF////////////////////
////
////        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
////
////        telemetry = new MultipleTelemetry(
////                telemetry,
////                dashboardTelemetry
////        );
////
////        pathTimer = new Timer();
////        autoTimer = new Timer();
////        shootTimer= new Timer();
////
////        follower = Constants.createFollower(hardwareMap);
////
////        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
////
////        buildPaths();
////        /* ---------------- WAIT ---------------- */
////        waitForStart();
////
////        autoTimer.resetTimer();
////        pathTimer.resetTimer();
////        shootTimer.resetTimer();
////        follower.setStartingPose(startPose);
////        /* ---------------- MAIN LOOP ---------------- */
////        while (opModeIsActive() && pathState != PathState.END) {
//////            panelsTelemetry.update();
////            telemetry.update();
////            follower.update();
////
//////            autoshoot();
////            statePathUpdate();
////
////
////        }
////    }
////
////
////
////
////    private void buildPaths() {
////        driveStartShoot = follower.pathBuilder()
////                .addPath(new BezierLine(startPose, shootPose))
////                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
////                .build();
////        driveReadyFirstPickup = follower.pathBuilder()
////                .addPath(new BezierCurve(shootPose, readyFirstPickupPose))
////                .setLinearHeadingInterpolation(shootPose.getHeading(), readyFirstPickupPose.getHeading())
////                .build();
////        driveFirstPickup = follower.pathBuilder()
////                .addPath(new BezierLine(readyFirstPickupPose, firstPickupPose))
////                .setLinearHeadingInterpolation(readyFirstPickupPose.getHeading(), firstPickupPose.getHeading())
////                .build();
////        driveFirstPickupShoot = follower.pathBuilder()
////
////        driveReadySecondPickup = follower.pathBuilder()
////
////        driveSecondPickup = follower.pathBuilder()
////
////
////        driveReadyThirdPickup = follower.pathBuilder()
////
////        driveThirdPickup = follower.pathBuilder()
////
////        driveThirdPickupShoot = follower.pathBuilder()
////
////
////        driveReadyOpenGatePickup = follower.pathBuilder()
////                .addPath(new BezierLine())
////                .build();
////        driveOpenGatePickup = follower.pathBuilder()
////                .addPath(new BezierLine(
////                .build();
////        driveOpenGatePickupShoot = follower.pathBuilder()
////                .addPath(;
////
////        driveOffline = follower.pathBuilder()
////                .addPath(;
////
////        driveOfflineofFirst = follower.pathBuilder()
////            .;
////    }
////
////    /* ---------------- STATE MACHINE ---------------- */
/////// /////////////////////////keep working here/////////////////////////12172025///////////
////    private void setPathState(PathState newState) {
////        pathState = newState;
////        pathTimer.resetTimer();
////    }
//    private void statePathUpdate() {
//        switch (pathState) {
//            case DRIVE_START_POS_SHOOT_POS:
//                follower.followPath(driveStartShoot, 0.65, true);
//                setPathState(PathState.DRIVE_TO_SHOOT_WAIT);
//                break;
//
//            case DRIVE_TO_SHOOT_WAIT:
//                if (!follower.isBusy()) {
//                    setPathState(PathState.SHOOT_PRELOAD);
//                }
//                break;
//
//            case SHOOT_PRELOAD:
//
//               if (autoShootState == AutoShootState.DONE) {
//                   follower.followPath(driveReadySecondPickup, 0.65, true);
//                   isShooterAtSpeed = false;
//                   setPathState(PathState.DRIVE_READY_SECOND_PICKUPT_POS);
//                    }
//               break;
//
//            case DRIVE_READY_SECOND_PICKUPT_POS:
//
//                break;
//
//            case SECOND_PICKUP:
//
//                break;
//            case
//                break;
//
//            case
//                break;
//
//            case
//                break;
//
//            case SHOOT_SECOND_PICKUP:
//
//
//                break;
//
//            case DRIVE_READY_THIRD_PICKUPT_POS:
//                if (!follower.isBusy()) {
//                    follower.followPath(driveThirdPickup, 0.5, true);
//                    setPathState(PathState.THIRD_PICKUP);
//                }
//                break;
//
//            case THIRD_PICKUP:
//
//                break;
//
//            case DRIVE_BACK_THIRD_PICKUPT_POS:
//                if (!follower.isBusy()) {
//                    setPathState(PathState.SHOOT_THIRD_PICKUP);
//                    autoShootState = AutoShootState.IDLE;
//                }
//                break;
//
//            case SHOOT_THIRD_PICKUP:
//
//
//                break;
//
//            case DRIVE_READY_FIRST_PICKUP_POS:
//
//                break;
//
//            case FIRST_PICKUP:
//                if (pathTimer.getElapsedTimeSeconds() < 1.8) {
//
//                } else {
//                    follower.followPath(driveFirstPickupShoot);
//
//                    setPathState(PathState.DRIVE_BACK_FIRST_SHOOT_POS);
//                }
//                break;
//
//            case DRIVE_BACK_FIRST_SHOOT_POS:
//                if (!follower.isBusy()) {
//                    setPathState(PathState.SHOOT_FIRST_PICKUP);
//                    autoShootState = AutoShootState.IDLE;
//                }
//                break;
//
//            case SHOOT_FIRST_PICKUP:
//                if (autoShootState == AutoShootState.DONE) {
//                    follower.followPath(driveOffline, 0.75, true);
//                    isShooterAtSpeed = false;
//                    setPathState(PathState.DRIVE_OFFLINE);
//                }
//                break;
//
//            case DRIVE_OFFLINE:
//                if (!follower.isBusy()) {
//                    setPathState(PathState.END);
//                }
//                break;
//        }
//    }
////}
