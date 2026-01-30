package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
// 姿态
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// Limelight
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
// 位姿
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@Autonomous(name = "AAA working on turret, tune pidf follow done Red9long")
public class Red9Long extends LinearOpMode {
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
    private static final double Far_SHOOTER_TARGET_RPM = 350;  //  350RPM---2237
//   private static final double Close_SHOOTER_TARGET_RPM = 800;//  400RPM---2,557.33333333333333
//    private static final double Med_SHOOTER_TARGET_RPM = 1300;   //1598 white tri a little bit too far//  250RPM---1586.67
//    private static final double Far_SHOOTER_TARGET_RPM = 2237;  //  350RPM---2237
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
    public float  intakePowerIntake=0.95f;
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
//    ShooterSubsystem shooterSubsystem;
//    FlywheelSubsystem flywheelSubsystem;
//    FeederSubsystem feederSubsystem;
//    IntakeSubsystem intakeSubsystem;
    private PathChain driveStartShoot, driveReadyFirstPickup, driveFirstPickup,
            driveFirstPickupShoot, driveReadySecondPickup,
            driveSecondPickup, driveSecondPickupShoot, driveOffline;
    public enum PathState {
        DRIVE_START_POS_SHOOT_POS,
        DRIVE_TO_SHOOT_WAIT,
        SHOOT_PRELOAD,
        DRIVE_READY_FIRST_PICKUP_POS,
        FIRST_PICKUP,
        DRIVE_BACK_FIRST_SHOOT_POS,
        SHOOT_FIRST_PICKUP,
        DRIVE_READY_SECOND_PICKUP,
        SECOND_PICKUP,
        DRIVE_BACK_SECOND_PICKUP,
        SHOOT_SECOND_PICKUP,
        DRIVE_OFFLINE,
        END
    }
    private final Pose startPose = new Pose(92, 6.25, Math.toRadians(0)); // Start Pose further zone of our robot.
    private final Pose shootPose = new Pose(92, 92.25, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose scoreEnd = new Pose(92, 92.25, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose readyFirstPickupPose = new Pose(92, 82.25, Math.toRadians(0)); // PPG  Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose readySecondPickupPose = new Pose(92, 60.25, Math.toRadians(0)); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose readyThirdPickupPose = new Pose(92, 36.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose firstPickupPose = new Pose(122, 82.25, Math.toRadians(0)); // PPG  Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose secondPickupPose = new Pose(122, 60.25, Math.toRadians(0)); // PGP Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose thirdPickupPose = new Pose(122, 36.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    //    private final Pose PARKPose = new Pose(120, 92.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose offlinePose = new Pose(110, 92.25, Math.toRadians(0)); // GPP Lowest (Third Set) of Artifacts from the Spike Mark.
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
//        robot.imu.resetYaw();
        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        FtcDashboard Dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = Dashboard.getTelemetry();
        // 设置 Dashboard 更新频率
        Dashboard.setTelemetryTransmissionInterval(100); // 100ms 更新一次

        pathTimer = new Timer();
        autoTimer = new Timer();
        shootTimer= new Timer();

        follower = Constants.createFollower(hardwareMap);

        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
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
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        autoTimer.resetTimer();
        pathTimer.resetTimer();
        shootTimer.resetTimer();
        follower.setStartingPose(startPose);
        /* ---------------- MAIN LOOP ---------------- */
        while (opModeIsActive() && pathState != PathState.END) {
            panelsTelemetry.update();
            follower.update();
//            autoshoot();

            statePathUpdate();
/// ///////////////////////////////////////////////debug PIDF////////////////////
//            updateShooter();
//            autoshoot();

            telemetry.addData("State", pathState);
            telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();

//            YawPitchRollAngles orientation =robot.imu.getRobotYawPitchRollAngles();
//            limelight.updateRobotOrientation(orientation.getYaw());
//            LLResult llResult= limelight.getLatestResult();
//            if(llResult!=null && llResult.isValid()){
//                Pose3D botPose=llResult.getBotpose_MT2();
//                telemetry.addData("Tx",llResult.getTx());
//                telemetry.addData("Ty",llResult.getTy());
//                telemetry.addData("Ta",llResult.getTa());
//
//
//            }
        }
    }



    /// ////////////////////////////////////////////////////


//    public static double flywheelSpeed(double goalDist){ return Mathfunctions.clamp(0.0204772 * Math,pow(goalDist, 2)+ 0.643162 * goalDist + 712.90909, lower 0, upper: 1400)+ flywheeloffset;}
//
//    public static double hoodAngle(double goalDist){ return MathFunctions.clamp(-2.34831e-7 * Math.pow(goalDist, 3)+ 0.0000936893 * math.pow(goalDist, 2)- 0.0165033*goalDist + 1.25724,lower: 0.11, upper: 0.904)+ hoodoffset;
//    }
//    public void updateShooter() {
//
//            checkShooterVelocity();
//            if (!robot.MasterShooterMotorL.isBusy()){
//                startShooter();
//            }
//
//        // 手动射击触发 - 按下A键射击（仅在速度达标时有效）
////        if (gamepad1.right_bumper && isShooterAtSpeed && !fireRequested)
//        if (gamepad1.right_bumper) {
//            fireRequested = true;
//            executeFireSequence();
//            robot.IntakeMotor.setPower(intakePowerShoot);
//        }
//        // 停止射击电机 - 按下B键
//        if (gamepad1.b) {
//            stopShooter();
//            robot.IntakeMotor.setPower(intakePowerOff);
//            fireRequested = false;
//        }
//
//        // 如果射击电机未运行，确保重置状态
//        if (!gamepad1.right_bumper && !robot.MasterShooterMotorL.isBusy()) {
//            isShooterAtSpeed = false;
//            fireRequested = false;
//        }
//        // 更新射击系统遥测数据（关键！）
//        updateShooterTelemetry();
//    }

    public void autoIntake() {
           // 吸入
            robot.IntakeMotor.setPower(intakePowerIntake);
            robot.MasterShooterMotorL.setPower(ShooterMotorHold);
            robot.SlaveShooterMotorR.setPower(ShooterMotorHold);
            telemetry.addData("intakePowerIntake", intakePowerIntake);
            telemetry.update();
    }

    enum AutoShootState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        DONE
    }

    private AutoShootState autoShootState = AutoShootState.IDLE;

    public void autoshoot() {

        double currentRPM = (Math.abs(robot.MasterShooterMotorL.getVelocity())*60)/(28);//60/(28*13.7)
        double targetRPM = ShooterPIDFConfig.targetRPM;

        switch (autoShootState) {

            case IDLE:
                startShooter();
                shooterStarted = true;
                shootTimer.resetTimer();
                autoShootState = AutoShootState.SPINNING_UP;
                break;

            case SPINNING_UP:
                if (Math.abs(currentRPM - targetRPM) <= ShooterPIDFConfig.toleranceofRPM) {
                    shootTimer.resetTimer(); // ⭐ 只在“到速”瞬间 reset
                    robot.IntakeMotor.setPower(intakePowerShoot);
                    autoShootState = AutoShootState.FEEDING;
                }
                break;

            case FEEDING:
                if (shootTimer.getElapsedTimeSeconds()  >= 2.0) {
                    stopShooter();
                    stopIntake();
                    autoShootState = AutoShootState.DONE;
                }
                break;

            case DONE:
                // 什么都不做，防止重复执行
                break;
        }
    }

    private void startShooter() {
        robot.IntakeMotor.setPower(0);
        if (robot.MasterShooterMotorL instanceof DcMotorEx) {
            DcMotorEx shooter = (DcMotorEx) robot.MasterShooterMotorL;
            // 直接使用setVelocity，它会使用已配置的PIDF
            shooter.setVelocity((ShooterPIDFConfig.targetRPM)*28/60);//28*13.7/60)
            double MasterShooterMotorLPower = robot.MasterShooterMotorL.getPower();
            double SlaveShooterMotorRPower = calculateOptimalSlavePower(MasterShooterMotorLPower);
            robot.SlaveShooterMotorR.setPower(SlaveShooterMotorRPower);

        }
        shooterStarted=true;

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
        public static double toleranceofRPM = 80;    // 转速容差 5RPM---30TPS
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
            double targetVelocity = (ShooterPIDFConfig.targetRPM)*28/60; //28*13.7/60
            double toleranceofVelocity = ShooterPIDFConfig.tolerance;
            double shooterPower = robot.MasterShooterMotorL.getPower();
            double shooterCurrent = robot.MasterShooterMotorL.getCurrent(CurrentUnit.AMPS); // 如果有电流传感器
            telemetry.addLine("=== SHOOTER PIDF TUNING ===");
            telemetry.addLine("Shooter Velocity");
            telemetry.addData("Target RPM",ShooterPIDFConfig.targetRPM);
            telemetry.addData("targetVelocity ",targetVelocity);
            telemetry.addData("CurrentVelocity", "%.0f", currentVelocity);
            telemetry.addData("Error Velocity", "%.1f", targetVelocity - currentVelocity);

            telemetry.addData("At Speed?", isShooterAtSpeed ? "YES" : "NO");
            telemetry.addData("Power", "%.2f", shooterPower);
            telemetry.update();

            // PIDF 参数值
            telemetry.addLine("=== PIDF PARAMETERS ===");
            telemetry.addData("kP", "%.4f", ShooterPIDFConfig.kP);
            telemetry.addData("kI", "%.4f", ShooterPIDFConfig.kI);
            telemetry.addData("kD", "%.4f", ShooterPIDFConfig.kD);
            telemetry.addData("kF", "%.4f", ShooterPIDFConfig.kF);
            telemetry.addData("Tolerance", "%.0f RPM", ShooterPIDFConfig.tolerance);

            // 从电机状态
            telemetry.addLine("=== SLAVE MOTOR ===");
            telemetry.addData("Slave Power", "%.2f", robot.SlaveShooterMotorR.getPower());
            telemetry.addData("Slave RPM", "%.0f", robot.SlaveShooterMotorR.getVelocity());

            // 射击状态
            telemetry.addLine("=== SHOOTING STATUS ===");
            telemetry.addData("Fire Requested", fireRequested ? "YES" : "NO");
            telemetry.addData("LED Color", isShooterAtSpeed ? "GREEN" : "RED");

            // PIDF 参数值
            telemetry.addLine("=== PIDF PARAMETERS ===");
            telemetry.addData("kP", "%.4f", ShooterPIDFConfig.kP);
            telemetry.addData("kI", "%.4f", ShooterPIDFConfig.kI);
            telemetry.addData("kD", "%.4f", ShooterPIDFConfig.kD);
            telemetry.addData("kF", "%.4f", ShooterPIDFConfig.kF);
            telemetry.addData("Tolerance", "%.0f RPM", ShooterPIDFConfig.tolerance);

            // 从电机状态
            telemetry.addLine("=== SLAVE MOTOR ===");
            telemetry.addData("Slave Power", "%.2f", robot.SlaveShooterMotorR.getPower());
            telemetry.addData("Slave RPM", "%.0f", robot.SlaveShooterMotorR.getVelocity());


            // 检查是否在容差范围内
            if (Math.abs(currentVelocity - targetVelocity) <= toleranceofVelocity) {
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
        telemetry.addData("Target RPM", "%.0f", ShooterPIDFConfig.targetRPM);
        telemetry.addData("Current RPM", "%.0f", shooterVelocity);
        telemetry.addData("Error", "%.0f RPM", Math.abs(ShooterPIDFConfig.targetRPM - shooterVelocity));
        telemetry.addData("At Speed?", isShooterAtSpeed ? "YES" : "NO");
        telemetry.addData("Power", "%.2f", shooterPower);

        // PIDF 参数值
        telemetry.addLine("=== PIDF PARAMETERS ===");
        telemetry.addData("kP", "%.4f", ShooterPIDFConfig.kP);
        telemetry.addData("kI", "%.4f", ShooterPIDFConfig.kI);
        telemetry.addData("kD", "%.4f", ShooterPIDFConfig.kD);
        telemetry.addData("kF", "%.4f", ShooterPIDFConfig.kF);
        telemetry.addData("Tolerance", "%.0f RPM", ShooterPIDFConfig.tolerance);

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
        shooterStarted=false;
    }

    private void stopIntake() {
        robot.IntakeMotor.setPower(intakePowerOff);
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
        driveReadyFirstPickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, readyFirstPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyFirstPickupPose.getHeading())
                .build();
//        driveReadyFirstPickup = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, firstPickupCP, readyFirstPickupPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), readyFirstPickupPose.getHeading())
//                .build();
        driveFirstPickup = follower.pathBuilder()
                .addPath(new BezierLine(readyFirstPickupPose, firstPickupPose))
                .setLinearHeadingInterpolation(readyFirstPickupPose.getHeading(), firstPickupPose.getHeading())
                .build();
        driveFirstPickupShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, shootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose.getHeading())
                .build();
        driveReadySecondPickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, readySecondPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readySecondPickupPose.getHeading())
                .build();
//        driveReadySecondPickup = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, secondPickupCP, readySecondPickupPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), readySecondPickupPose.getHeading())
//                .build();
        driveSecondPickup = follower.pathBuilder()
                .addPath(new BezierLine(readySecondPickupPose, secondPickupPose))
                .setLinearHeadingInterpolation(readySecondPickupPose.getHeading(), secondPickupPose.getHeading())
                .build();
        driveSecondPickupShoot = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose, shootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
                .build();
//        driveSecondPickupShoot = follower.pathBuilder()
//                .addPath(new BezierCurve(secondPickupPose, secondPickupCP, shootPose))
//                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
//                .build();
        driveOffline = follower.pathBuilder()
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
                follower.followPath(driveStartShoot, 0.8, true);
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
                   follower.followPath(driveReadyFirstPickup, 0.8, true);
                   setPathState(PathState.DRIVE_READY_FIRST_PICKUP_POS);
                    }
               break;
            case DRIVE_READY_FIRST_PICKUP_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveFirstPickup, 0.5, true);
                    setPathState(PathState.FIRST_PICKUP);
                }
                break;
            case FIRST_PICKUP:
                if (pathTimer.getElapsedTimeSeconds() < 2.5) {
            autoIntake();

                } else {
                    follower.followPath(driveFirstPickupShoot);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_FIRST_SHOOT_POS);
                }
                break;
            case DRIVE_BACK_FIRST_SHOOT_POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIRST_PICKUP);
                }
                break;
            case SHOOT_FIRST_PICKUP:
                autoShootState = AutoShootState.IDLE;
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.followPath(driveReadySecondPickup, 0.8, true);
                    setPathState(PathState.DRIVE_READY_SECOND_PICKUP);
                }
                break;

            case DRIVE_READY_SECOND_PICKUP:
                if (!follower.isBusy()) {
                    follower.followPath(driveSecondPickup, 0.8, true);
                    setPathState(PathState.SECOND_PICKUP);
                }
                break;
            case SECOND_PICKUP:
                if (pathTimer.getElapsedTimeSeconds() < 2.5) {
                    autoIntake();
                } else {
                    follower.followPath(driveSecondPickupShoot);
                    stopShooter();
                    stopIntake();
                    setPathState(PathState.DRIVE_BACK_SECOND_PICKUP);
                }
                break;
            case DRIVE_BACK_SECOND_PICKUP:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SECOND_PICKUP);
                }
                break;
            case SHOOT_SECOND_PICKUP:
                autoShootState = AutoShootState.IDLE;
                autoshoot();
                if (autoShootState == AutoShootState.DONE) {
                    follower.followPath(driveOffline, 0.8, true);
                    setPathState(PathState.DRIVE_OFFLINE);
                }
                break;
//                if (pathTimer.getElapsedTimeSeconds() > 5) {
//                    follower.followPath(driveOffline);
//                    setPathState(PathState.DRIVE_OFFLINE);
//                }
//                break;

            case DRIVE_OFFLINE:
                if (!follower.isBusy()) {
                    setPathState(PathState.END);
                }
                break;
        }
    }
}
