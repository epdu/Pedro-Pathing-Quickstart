package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpQualifier.blockageblockTele;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpQualifier.blockagereleaseTele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
// 姿态
// Limelight
// 位姿
@Autonomous(name = "RED Near opengate with intake TWELVE  park after third loaded V1 Red9short")
// intake second role with open gate , park at  136, 36.25 with three loaded
//. 3 ARTIFACTS on each SPIKE MARK arranged as follows:
//i.Near (audience side): GPP
//ii. Middle: PGP
//iii. Far (GOAL side): PPG
public class Red21nearopengatewithintake extends LinearOpMode {
    HardwareQualifier robot = new HardwareQualifier();
    private Limelight3A limelight;
    private volatile boolean isRunning = true;
    private boolean shooterStarted=false;
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();
    private Timer autoTimer, pathTimer,shootTimer;
    private static final double VELOCITY_TOLERANCE = 30; // RPM容差，可根据测试调整
    // 状态变量
    private boolean isShooterAtSpeed = false;
    private boolean wasShooterAtSpeed = false; // 用于检测状态变化
    private boolean firstPickupCompleted = false;
    private boolean fireRequested = false;
    // LED颜色常量（根据你的LED库调整）
    private final String LED_COLOR_READY = "GREEN";
    private final String LED_COLOR_ACCELERATING = "YELLOW";
    private final String LED_COLOR_OFF = "RED";
//  RPM = (TPS * 60秒) / 每转ticks数
//  return (tps * 60.0) / ticksPerRevolution;  28*13.7     28*13.7-----28 6000RPM
//  TPS=RPM/60*ticksPerRevolution=RPM*28*13.7/60；
    private static final double Close_SHOOTER_TARGET_RPM = 100;//  400RPM---2,557.33333333333333
//    private static final double Med_SHOOTER_TARGET_RPM = 204;   //1598 white tri a little bit too far//  250RPM---1586.67
    private static final double Med_SHOOTER_TARGET_RPM = 2785;   //1598 white tri a little bit too far//  250RPM---1586.67//150-100 too big
//    private static final double Med_SHOOTER_TARGET_Velocity = 1300;
    private static final double Far_SHOOTER_TARGET_RPM = 350;  //  350RPM---2237
//   private static final double Close_SHOOTER_TARGET_RPM = 800;//  400RPM---2,557.33333333333333
//    private static final double Med_SHOOTER_TARGET_RPM = 1300;   //1598 white tri a little bit too far//  250RPM---1586.67
//    private static final double Far_SHOOTER_TARGET_RPM = 2237;  //  350RPM---2237
    ///////////////////////////////
    private static final double Med_SHOOTER_TARGET_Velocity = 1150; //1598 white tri a little bit too far//  250RPM---1586.67//150-100 too big
    public float  intakePowerIntake=0.85f;//0.95
    public float  intakePowerShoot=0.9f;//0.9
    public float DriveTrains_ReducePOWER=0.75f;
    public float DriveTrains_smoothTurn=0.55f;
//    public String fieldOrRobotCentric = "robot";
    public String fieldOrRobotCentric = "field";
    private double powerMultiplier = 0.9;
    boolean move = false;
    int controlMode = 1;
//    public float  intakePowerIntake=0.75f;//0.95
//    public float  intakePowerShoot=0.85f;//0.9
//    public float  intakePowerShoot=0.8f;//0.9
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
    // Other variables
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathStatePreload;
    private int pathStatePPG; // Current state machine value
    private int pathStatePGP; // Current state machine value
    private int pathStateGPP; // Current state machine value
    private boolean hasPathStarted = false;
    private PathState pathState;
    private AutoShootState autoShootState;
//    ShooterSubsystem shooterSubsystem;
//    FlywheelSubsystem flywheelSubsystem;
//    FeederSubsystem feederSubsystem;
//    IntakeSubsystem intakeSubsystem;
private PathChain driveStartShoot;
    private PathChain driveReadyThirdSpikePickup, driveThirdSpikePickup, driveThirdSpikePickupShoot;
    private PathChain driveReadyOpenGatePickup, driveOpenGatePickup, driveOpenGatePickupShoot;
    private PathChain driveReadySecondSpikePickup,driveSecondSpikePickup, driveSecondSpikePickupShoot;
    private PathChain driveReadyFirstSpikePickup, driveFirstSpikePickup,driveFirstSpikePickupShoot;
    private PathChain driveOffline,driveOfflineofFirst;

    public enum PathState {
        DRIVE_START_POS_SHOOT_POS,
        DRIVE_TO_SHOOT_WAIT,
        SHOOT_PRELOAD,
        DRIVE_READY_THIRD_SPIKE_PICKUPT_POS,
        THIRD_SPIKE_PICKUP,
        DRIVE_BACK_THIRD_SPIKE_PICKUPT_POS,
        SHOOT_THIRD_SPIKE_PICKUP,
        DRIVE_READY_SECOND_SPIKE_PICKUPT_POS,
        SECOND_SPIKE_PICKUP,
        DRIVE_BACK_SECOND_SPIKE_PICKUPT_POS,
        SHOOT_SECOND_SPIKE_PICKUP,
        DRIVE_READY_FIRST_OPEN_GATE_POS,
        FIRST_OPEN_GATE,
        FIRST_OPEN_GATE_INTAKE,
        DRIVE_BACK_FIRST_OPEN_GATE_PICKUPT_POS,
        SHOOT_FIRST_OPEN_GATE_PICKUP,
        DRIVE_READY_SECOND_OPEN_GATE_POS,
        SECOND_OPEN_GATE,
        SECOND_OPEN_GATE_INTAKE,
        DRIVE_BACK_SECOND_OPEN_GATE_PICKUPT_POS,
        SHOOT_SECOND_OPEN_GATE_PICKUP,
        DRIVE_READY_THIRD_OPEN_GATE_POS,
        THIRD_OPEN_GATE,
        THIRD_OPEN_GATE_INTAKE,
        DRIVE_BACK_THIRD_OPEN_GATE_PICKUPT_POS,
        SHOOT_THIRD_OPEN_GATE_PICKUP,
        DRIVE_READY_FIRST_SPIKE_PICKUP_POS,
        FIRST_SPIKE_PICKUP,
        DRIVE_BACK_FIRST_SPIKE_SHOOT_POS,
        SHOOT_FIRST_SPIKE_PICKUP,
        DRIVE_OFFLINE,
        END
    }
    private final Pose startPose = new Pose(127.5,118.75, Math.toRadians(0)); // Start Pose further zone of our robot.
    private final Pose shootPose = new Pose(92, 92.25, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose scoreEnd = new Pose(92, 92.25, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose readyThirdSpikePickupPose = new Pose(92, 86.25, Math.toRadians(0)); // PPG  Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose readySecondSpikePickupPose = new Pose(92, 62.25, Math.toRadians(0)); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose readyFirstSpikePickupPose = new Pose(92, 36.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose readyOpenGatePickupPose = new Pose(124.5,67.5,Math.toRadians(0)); // 63--64  PGP Middle (Second Set) of Artifacts from the Spike Mark.
/// //////////////////////////////////
    private final Pose readyOpenGateForPickPickupPose = new Pose(124.5,67.5,Math.toRadians(18));
    private final Pose thirdSpikePickupPose = new Pose(127, 86.25, Math.toRadians(0)); // PPG  Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose secondSpikePickupPose = new Pose(133, 62.25, Math.toRadians(0)); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose secondSpikePickupCP = new Pose(72, 76, Math.toRadians(0));
    private final Pose openGatePickupCP = new Pose(72, 76, Math.toRadians(0));
    ////////////////////////
    private final Pose openGateForPickPickupCP = new Pose(72, 76, Math.toRadians(18));
    private final Pose secondSpikePickupPoseControlPoint = new Pose(125, 64.25, Math.toRadians(0)); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose firstSpikePickupPose = new Pose(134, 36.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose openGatePickupPose = new Pose(133, 69, Math.toRadians(0));  //63--61
    /// /////////////////////////////////////////
    private final Pose openGateForPickPickupPose = new Pose(133, 69, Math.toRadians(18));
    private final Pose openGateForPickPickupPoseAdjust = new Pose(124.5,67.5,Math.toRadians(26));

    //    private final Pose PARKPose = new Pose(120, 92.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose offlinePose = new Pose(112, 92.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    // Initialize variables for paths
//    private final Pose offlinePoseofFirst = new Pose(136, 36.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    // Initialize variables for paths
    @Override
    public void runOpMode() {

        /* ---------------- INIT ---------------- */
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
///////////////////////////////////////////////////////
//
//        Robot.alliance = Alliance.RED;
//        Robot.sendHardwareMap(hardwareMap);
        robot.init(hardwareMap);
//        PanelsTelemetry.INSTANCE.init(hardwareMap);
//        robot.imu.resetYaw();
        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.BlockageArm.setPosition(blockageblockposition);////switch with blockage case with blockage
        robot.BlockageArm.setPosition(blockageblockTele); //switch with blockage case with blockage need command
//        follower.setStartingPose(startPose);
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        limelight.pipelineSwitch(0);
//        /*
//         * Starts polling for data.
//         */
//        limelight.start();// 需要手动开启吗？
/// ///////////////////////////////////////////////debug PIDF////////////////////
        initShooterPIDF();
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        // 设置 Dashboard 更新频率
//        dashboard.setTelemetryTransmissionInterval(100); // 100ms 更新一次
//        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());


        telemetry = new MultipleTelemetry(
                telemetry,
                dashboardTelemetry
        );

//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        autoTimer = new Timer();
        shootTimer= new Timer();

        follower = Constants.createFollower(hardwareMap);

        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
        autoShootState = AutoShootState.IDLE;
        buildPaths();
//        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
//        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
//        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
//        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
//        shooterSubsystem.init();
//        flywheelSubsystem.init();
//        feederSubsystem.init();
//        intakeSubsystem.init();
//        telemetry.addLine("Ready");
//        telemetry.update();

        /* ---------------- WAIT ---------------- */
        waitForStart();

        autoTimer.resetTimer();
        pathTimer.resetTimer();
        shootTimer.resetTimer();
        follower.setStartingPose(startPose);
        /* ---------------- MAIN LOOP ---------------- */
        while (opModeIsActive() && pathState != PathState.END) {
//            panelsTelemetry.update();
            telemetry.update();
            follower.update();
            updateShooterTelemetry();
//            autoshoot();
            statePathUpdate();

//////////////////////////////////////////////////debug PIDF////////////////////
//            YawPitchRollAngles orientation =robot.imu.getRobotYawPitchRollAngles();
//            limelight.updateRobotOrientation(orientation.getYaw());
//            LLResult llResult= limelight.getLatestResult();
//            if(llResult!=null && llResult.isValid()){
//                Pose3D botPose=llResult.getBotpose_MT2();
//                telemetry.addData("Tx",llResult.getTx());
//                telemetry.addData("Ty",llResult.getTy());
//                telemetry.addData("Ta",llResult.getTa());
//            }




        }
    }

////////////////////////////////////////////////////

    public void autoIntake() {
           // 吸入
            robot.IntakeMotorL.setPower(intakePowerIntake);
            robot.IntakeMotorR.setPower(intakePowerIntake);

            robot.BlockageArm.setPosition(blockageblockTele);


//            robot.MasterShooterMotorL.setPower(ShooterMotorHold);//switch with blockage case with blockage need command
//            robot.SlaveShooterMotorR.setPower(ShooterMotorHold); //switch with blockage case with blockage need command
            telemetry.update();
    }

    enum AutoShootState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        DONE
    }

    public void autoshoot() {
        switch (autoShootState) {

            case IDLE:
                robot.BlockageArm.setPosition(blockageblockTele); //switch with blockage case with blockage
                startShooter();
                shooterStarted = true;
                isShooterAtSpeed = false;
                shootTimer.resetTimer();
                autoShootState = AutoShootState.SPINNING_UP;
                break;

            case SPINNING_UP:
                  robot.BlockageArm.setPosition(blockagereleaseTele); //switch with blockage case with blockage
//                double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());//60/(28)
//                double targetVelocity = ShooterPIDFConfig.targetVelocity;
//                if ((!isShooterAtSpeed) && (Math.abs(currentVelocity - targetVelocity) <= ShooterPIDFConfig.toleranceofVelocity)) {
                if ((Math.abs(Math.abs(robot.MasterShooterMotorL.getVelocity()) - ShooterPIDFConfig.targetVelocity) <= ShooterPIDFConfig.toleranceofVelocity)) {
                    robot.IntakeMotorL.setPower(intakePowerShoot);
                    robot.IntakeMotorR.setPower(intakePowerShoot);
//                    isShooterAtSpeed = true;
                    shootTimer.resetTimer();
                    autoShootState = AutoShootState.FEEDING;
                }

                break;

            case FEEDING:
                if (shootTimer.getElapsedTimeSeconds()  >= 1.6) {
                    // from 1.9---1.6
                    stopShooter();
                    stopIntake();
                    robot.BlockageArm.setPosition(blockageblockTele); //switch with blockage case with blockage
                    autoShootState = AutoShootState.DONE;
                }
                break;


//            case FEEDING:
//                if (shootTimer.getElapsedTimeSeconds()  >= 2.2) {
//                    stopShooter();
//                    stopIntake();
//                    robot.BlockageArm.setPosition(blockageblockposition);
////                    isShooterAtSpeed = false;
//                    autoShootState = AutoShootState.DONE;
//                }
//                break;

            case DONE:
                // 什么都不做，防止重复执行
                break;
        }
    }

private void startShooter() {
    robot.IntakeMotorL.setPower(0);
    robot.IntakeMotorR.setPower(0);
    if (robot.MasterShooterMotorL instanceof DcMotorEx) {
        DcMotorEx shooter = (DcMotorEx) robot.MasterShooterMotorL;
        DcMotorEx shooterR = (DcMotorEx) robot.SlaveShooterMotorR;
        // 直接使用setVelocity，它会使用已配置的PIDF
        shooter.setVelocity(Math.abs(ShooterPIDFConfig.targetVelocity));
        shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetVelocity));
    }
    shooterStarted=true;

}


/// ///////////////////////////////
@Config
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
//        public static double targetRPM =Med_SHOOTER_TARGET_RPM; // 目标转速
        public static double targetVelocity =Med_SHOOTER_TARGET_Velocity;
//        public static double toleranceofRPM = 100;    // 转速容差 5RPM---30TPS
        public static double toleranceofVelocity = 70;
        public static double tolerance = 50;

    }

    private void initShooterPIDF() {
        // 初始化时调用一次
        if (robot.MasterShooterMotorL instanceof DcMotorEx) {
            DcMotorEx shooter = (DcMotorEx) robot.MasterShooterMotorL;

            // 设置PIDF参数
            PIDFCoefficients pidf = new PIDFCoefficients(
                    ShooterPIDFConfig.kP,
                    ShooterPIDFConfig.kI,
                    ShooterPIDFConfig.kD,
                    ShooterPIDFConfig.kF
            );
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            telemetry.addData("Shooter PIDF", "Initialized");
            telemetry.addLine("Shooter PIDF");
            telemetry.addData("kP", pidf.p);
            telemetry.addData("kI", pidf.i);
            telemetry.addData("kD", pidf.d);
            telemetry.addData("kF", pidf.f);
            telemetry.update();

        }
    }


    /**
     * 检查射击电机速度（使用 Dashboard 调整的容差）
     */
//    private void checkShooterVelocity() {
//
//            double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());
//            double targetVelocity = (ShooterPIDFConfig.targetVelocity); //28*13.7/60
//            // targetVelocity
////            double targetVelocity = (ShooterPIDFConfig.targetRPM)*28/60; //28*13.7/60
//            double toleranceofVelocity = ShooterPIDFConfig.tolerance;
//            double shooterPower = robot.MasterShooterMotorL.getPower();
//            double shooterCurrent = robot.MasterShooterMotorL.getCurrent(CurrentUnit.AMPS); // 如果有电流传感器
//
//            // 检查是否在容差范围内
//            if (Math.abs(Math.abs(currentVelocity) - targetVelocity) <= toleranceofVelocity) {
//                isShooterAtSpeed = true;
//            } else {
//                isShooterAtSpeed = false;
//                fireRequested = false;
//            }
//
//    }


    /**
     * 更新射击系统遥测数据（用于 PIDF 调参）
     */
    private void updateShooterTelemetry() {
        // 射击电机状态
        telemetry.addLine("=== SHOOTER PIDF TUNING ===");
        double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());
        double currentVelocityR = Math.abs(robot.SlaveShooterMotorR.getVelocity());
//        double targetVelocity = ShooterPIDFConfig.targetVelocity;
//        double tolerance = ShooterPIDFConfig.tolerance;
//        telemetry.addData("VEL raw", robot.MasterShooterMotorL.getVelocity());
//        telemetry.addData("VEL target", ShooterPIDFConfig.targetVelocity);
        telemetry.addData("VEL error", ShooterPIDFConfig.targetVelocity - robot.MasterShooterMotorL.getVelocity());
        telemetry.addData("currentVelocity", currentVelocity);
        telemetry.addData("  shootTimer ", shootTimer.getElapsedTimeSeconds());
        telemetry.addData("currentVelocityR", currentVelocityR);
        telemetry.addData("ShooterPIDFConfig.targetVelocity", ShooterPIDFConfig.targetVelocity);
        telemetry.addData("ShooterPIDFConfig.toleranceofVelocity", ShooterPIDFConfig.toleranceofVelocity);
        telemetry.addData("ShooterPIDFConfig.tolerance", ShooterPIDFConfig.tolerance);
//        double shooterPower = robot.MasterShooterMotorL.getPower();
//        double shooterPowerr = robot.SlaveShooterMotorR.getPower();
//        telemetry.addData("At Speed?", isShooterAtSpeed ? "YES" : "NO");
//        telemetry.addData("ShooterPower", "%.2f", shooterPower);
//        telemetry.addData("ShooterPowerR", "%.2f", shooterPowerr);
        telemetry.addData("state AutoShootState",  autoShootState);
        telemetry.addData("State", pathState);
        telemetry.addData("isShooterAtSpeed", isShooterAtSpeed);
        telemetry.addData("State", pathState);
//        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        //        telemetry.addData(" robot.SlaveShooterMotorR.setPower", "%.2f",  (robot.MasterShooterMotorL.getPower()));


        /// /////////////////////////////////////////////////
//        // PIDF 参数值
//        telemetry.addLine("=== PIDF PARAMETERS ===");
//        telemetry.addData("kP", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kP);
////        telemetry.addData("kI", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kI);
////        telemetry.addData("kD", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kD);
//        telemetry.addData("kF", "%.4f", TeleOpQualifier.ShooterPIDFConfig.kF);
//        telemetry.addData("Tolerance", "%.0f RPM", TeleOpQualifier.ShooterPIDFConfig.tolerance);
//        telemetry.addData("MasterShooterMPower", "%.0f", robot.MasterShooterMotorL.getPower());
        // 从电机状态
//        telemetry.addLine("=== SLAVE MOTOR ===");
//        telemetry.addData("Slave Power", "%.2f", robot.SlaveShooterMotorR.getPower());
//        telemetry.addData("Slave RPM", "%.0f", robot.SlaveShooterMotorR.getVelocity());

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
        robot.IntakeMotorR.setPower(0);
        isShooterAtSpeed = false;
        fireRequested = false;
        shooterStarted=false;
    }

    private void stopIntake() {
        robot.IntakeMotorL.setPower(intakePowerOff);
        robot.IntakeMotorR.setPower(intakePowerOff);
        robot.MasterShooterMotorL.setPower(ShooterMotorOff);
        robot.SlaveShooterMotorR.setPower(ShooterMotorOff);
        // Non-blocking delay to prevent rapid mode switching
        telemetry.addData("intakePowerOff", intakePowerOff);
        telemetry.update();

        shooterStarted=false;
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
        robot.IntakeMotorL.setPower(intakePowerShoot); // 全功率推出
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
        double speedRatio =Math.abs(slaveSpeed / (masterSpeed + 0.001)) ; // 避免除零

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


    ///
    ///
    ///
    /// ///////////////////////////////////////////////////
    /* ---------------- PATH BUILDING ---------------- */

    private void buildPaths() {
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveReadyFirstSpikePickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, readyFirstSpikePickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyFirstSpikePickupPose.getHeading())
                .build();
//        driveReadyFirstPickup = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, firstPickupCP, readyFirstPickupPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), readyFirstPickupPose.getHeading())
//                .build();
        driveFirstSpikePickup = follower.pathBuilder()
                .addPath(new BezierLine(readyFirstSpikePickupPose, firstSpikePickupPose))
                .setLinearHeadingInterpolation(readyFirstSpikePickupPose.getHeading(), firstSpikePickupPose.getHeading())
                .build();
        driveFirstSpikePickupShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstSpikePickupPose, shootPose))
                .setLinearHeadingInterpolation(firstSpikePickupPose.getHeading(), shootPose.getHeading())
                .build();


        driveReadySecondSpikePickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, secondSpikePickupCP,readySecondSpikePickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readySecondSpikePickupPose.getHeading())
                .build();
        driveSecondSpikePickup = follower.pathBuilder()
                .addPath(new BezierLine(readySecondSpikePickupPose, secondSpikePickupPose))
                .setLinearHeadingInterpolation(readySecondSpikePickupPose.getHeading(), secondSpikePickupPose.getHeading())
                .build();
        driveSecondSpikePickupShoot = follower.pathBuilder()
                .addPath(new BezierCurve(secondSpikePickupPose, secondSpikePickupCP,shootPose))
                .setLinearHeadingInterpolation(secondSpikePickupPose.getHeading(), shootPose.getHeading())
                .build();
//        driveSecondPickupShoot = follower.pathBuilder()
//                .addPath(new BezierCurve(secondPickupPose, secondPickupCP,shootPose))
//                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
//                .build();
//        driveSecondPickupOpengate = follower.pathBuilder()
//                .addPath(new BezierCurve(secondPickupPose, secondPickupCP,shootPose))
//                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
//                .build();

        driveReadyThirdSpikePickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, readyThirdSpikePickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyThirdSpikePickupPose.getHeading())
                .build();
        driveThirdSpikePickup = follower.pathBuilder()
                .addPath(new BezierLine(readyThirdSpikePickupPose, thirdSpikePickupPose))
                .setLinearHeadingInterpolation(readyThirdSpikePickupPose.getHeading(), thirdSpikePickupPose.getHeading())
                .build();
        driveThirdSpikePickupShoot = follower.pathBuilder()
                .addPath(new BezierCurve(thirdSpikePickupPose, shootPose))
                .setLinearHeadingInterpolation(thirdSpikePickupPose.getHeading(), shootPose.getHeading())
                .build();
        driveReadyOpenGatePickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, readyOpenGatePickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyOpenGatePickupPose.getHeading())
                .build();

//        driveReadyOpenGatePickup = follower.pathBuilder()
//                .addPath(new BezierLine(secondSpikePickupPose, readyOpenGatePickupPose))
//                .setLinearHeadingInterpolation(secondSpikePickupPose.getHeading(), readyOpenGatePickupPose.getHeading())
//                .build();
        driveOpenGatePickup = follower.pathBuilder()
                .addPath(new BezierLine(readyOpenGatePickupPose, openGatePickupPose))
                .setLinearHeadingInterpolation(readyOpenGatePickupPose.getHeading(), openGatePickupPose.getHeading())
                .build();
        driveOpenGatePickupShoot = follower.pathBuilder()
                .addPath(new  BezierCurve(openGatePickupPose,openGatePickupCP,shootPose))
                .setLinearHeadingInterpolation(openGatePickupPose.getHeading(), shootPose.getHeading())
                .build();
        /// //////////////////////////

//        driveReadyOpenGateFirstPickup = follower.pathBuilder()
//                .addPath(new BezierLine(secondSpikePickupPose, readyOpenGatePickupPose))
//                .setLinearHeadingInterpolation(secondSpikePickupPose.getHeading(), readyOpenGatePickupPose.getHeading())
//                .build();
        driveOpenGatePickup = follower.pathBuilder()
                .addPath(new BezierLine(readyOpenGatePickupPose, openGatePickupPose))
                .setLinearHeadingInterpolation(readyOpenGatePickupPose.getHeading(), openGatePickupPose.getHeading())
                .build();
        driveOpenGatePickupShoot = follower.pathBuilder()
                .addPath(new  BezierCurve(openGatePickupPose,openGatePickupCP,shootPose))
                .setLinearHeadingInterpolation(openGatePickupPose.getHeading(), shootPose.getHeading())
                .build();

        driveOffline = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, offlinePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), offlinePose.getHeading())
                .build();

        driveOfflineofFirst = follower.pathBuilder()
            .addPath(new BezierLine(shootPose, offlinePose))
            .setLinearHeadingInterpolation(shootPose.getHeading(), offlinePose.getHeading())
            .build();
    }

    /* ---------------- STATE MACHINE ---------------- */
/// /////////////////////////keep working here/////////////////////////12172025///////////
    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    private void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.followPath(driveStartShoot, 0.65, true);
                setPathState(PathState.DRIVE_TO_SHOOT_WAIT);
                break;

            case DRIVE_TO_SHOOT_WAIT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
             autoshoot();
               if (autoShootState == AutoShootState.DONE) {
                   follower.followPath(driveReadySecondSpikePickup, 0.65, true);
                   isShooterAtSpeed = false;
                   setPathState(PathState.DRIVE_READY_SECOND_SPIKE_PICKUPT_POS);
                    }
               break;

            case DRIVE_READY_SECOND_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveSecondSpikePickup, 0.5, true);
                    setPathState(PathState.SECOND_SPIKE_PICKUP);
                }
                break;

            case SECOND_SPIKE_PICKUP:
                if (pathTimer.getElapsedTimeSeconds() < 1.8) {
                    autoIntake();
                } else {
                    follower.followPath(driveSecondSpikePickupShoot);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_SECOND_SPIKE_PICKUPT_POS);
                }
                break;

            case DRIVE_BACK_SECOND_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SECOND_SPIKE_PICKUP);
                    autoShootState = AutoShootState.IDLE;
                }
                break;

            case SHOOT_SECOND_SPIKE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.followPath(driveReadyOpenGatePickup, 0.65, true);
                    isShooterAtSpeed = false;
                    setPathState(PathState.DRIVE_READY_FIRST_OPEN_GATE_POS);
                }
                break;

            case DRIVE_READY_FIRST_OPEN_GATE_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveOpenGatePickup, 0.5, true);
                    setPathState(PathState.FIRST_OPEN_GATE);
                }
                break;

            case FIRST_OPEN_GATE:
                if (pathTimer.getElapsedTimeSeconds() < .5) {
                    autoIntake();
                } else {
                    follower.followPath(driveOpenGatePickup);
//                    stopShooter();
//                    stopIntake();
                    setPathState(PathState.FIRST_OPEN_GATE_INTAKE);
                }
                break;

            case FIRST_OPEN_GATE_INTAKE:
                if (pathTimer.getElapsedTimeSeconds() < .9) {
                    autoIntake();
                } else {
                    follower.followPath(driveOpenGatePickupShoot);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_FIRST_OPEN_GATE_PICKUPT_POS);
                }
                break;

            case DRIVE_BACK_FIRST_OPEN_GATE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIRST_OPEN_GATE_PICKUP);
                    autoShootState = AutoShootState.IDLE;
                }
                break;

            case SHOOT_FIRST_OPEN_GATE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.followPath(driveReadyOpenGatePickup, 0.65, true);
                    isShooterAtSpeed = false;
                    setPathState(PathState. DRIVE_READY_SECOND_OPEN_GATE_POS);
                }
                break;

            case DRIVE_READY_SECOND_OPEN_GATE_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveOpenGatePickup, 0.5, true);
                    setPathState(PathState.SECOND_OPEN_GATE);
                }
                break;

            case SECOND_OPEN_GATE:
                if (pathTimer.getElapsedTimeSeconds() < .9) {
                    autoIntake();
                } else {
                    follower.followPath(driveOpenGatePickup);
//                    stopShooter();
//                    stopIntake();
                    setPathState(PathState.SECOND_OPEN_GATE_INTAKE);
                }
                break;

            case SECOND_OPEN_GATE_INTAKE:
                if (pathTimer.getElapsedTimeSeconds() < .9) {
                    autoIntake();
                } else {
                    follower.followPath(driveOpenGatePickupShoot);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_SECOND_OPEN_GATE_PICKUPT_POS);
                }
                break;

            case DRIVE_BACK_SECOND_OPEN_GATE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SECOND_OPEN_GATE_PICKUP);
                    autoShootState = AutoShootState.IDLE;
                }
                break;

            case SHOOT_SECOND_OPEN_GATE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.followPath(driveReadyOpenGatePickup, 0.65, true);
                    isShooterAtSpeed = false;
                    setPathState(PathState. DRIVE_READY_THIRD_OPEN_GATE_POS);
                }
                break;

            case DRIVE_READY_THIRD_OPEN_GATE_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveOpenGatePickup, 0.5, true);
                    setPathState(PathState.THIRD_OPEN_GATE);
                }
                break;

            case THIRD_OPEN_GATE:
                if (pathTimer.getElapsedTimeSeconds() < .9) {
                    autoIntake();
                } else {
                    follower.followPath(driveOpenGatePickup);
//                    stopShooter();
//                    stopIntake();
                    setPathState(PathState.THIRD_OPEN_GATE_INTAKE);
                }
                break;

            case THIRD_OPEN_GATE_INTAKE:
                if (pathTimer.getElapsedTimeSeconds() < .9) {
                    autoIntake();
                } else {
                    follower.followPath(driveOpenGatePickupShoot);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_THIRD_OPEN_GATE_PICKUPT_POS);
                }
                break;

            case DRIVE_BACK_THIRD_OPEN_GATE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_THIRD_OPEN_GATE_PICKUP);
                    autoShootState = AutoShootState.IDLE;
                }
                break;

            case SHOOT_THIRD_OPEN_GATE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.followPath(driveReadyOpenGatePickup, 0.65, true);
                    isShooterAtSpeed = false;
                    setPathState(PathState. DRIVE_READY_THIRD_SPIKE_PICKUPT_POS);
                }
                break;

            case DRIVE_READY_THIRD_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveThirdSpikePickup, 0.5, true);
                    setPathState(PathState.THIRD_SPIKE_PICKUP);
                }
                break;

            case THIRD_SPIKE_PICKUP:
                if (!firstPickupCompleted && pathTimer.getElapsedTimeSeconds() < 1.8) {
            autoIntake();
                } else {
                    follower.followPath(driveThirdSpikePickupShoot);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_THIRD_SPIKE_PICKUPT_POS);
                    firstPickupCompleted = true;
                }
                break;

            case DRIVE_BACK_THIRD_SPIKE_PICKUPT_POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_THIRD_SPIKE_PICKUP);
                    autoShootState = AutoShootState.IDLE;
                }
                break;

            case SHOOT_THIRD_SPIKE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.followPath(driveReadyFirstSpikePickup, 0.65, true);
                    isShooterAtSpeed = false;
                    setPathState(PathState.DRIVE_READY_FIRST_SPIKE_PICKUP_POS);
                }
                break;

            case DRIVE_READY_FIRST_SPIKE_PICKUP_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveFirstSpikePickup, 0.5, true);
                    setPathState(PathState.FIRST_SPIKE_PICKUP);
                }
                break;

            case FIRST_SPIKE_PICKUP:
                if (pathTimer.getElapsedTimeSeconds() < 1.8) {
                    autoIntake();
                } else {
                    follower.followPath(driveFirstSpikePickupShoot);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_FIRST_SPIKE_SHOOT_POS);
                }
                break;

            case DRIVE_BACK_FIRST_SPIKE_SHOOT_POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIRST_SPIKE_PICKUP);
                    autoShootState = AutoShootState.IDLE;
                }
                break;

            case SHOOT_FIRST_SPIKE_PICKUP:
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.followPath(driveOffline, 0.75, true);
                    isShooterAtSpeed = false;
                    setPathState(PathState.DRIVE_OFFLINE);
                }
                break;

            case DRIVE_OFFLINE:
                if (!follower.isBusy()) {
                    setPathState(PathState.END);
                }
                break;
        }
    }
}
