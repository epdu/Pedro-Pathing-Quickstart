//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//
//@Config  // 添加这个注解，让 Dashboard 可以调整参数
//@TeleOp(name = "AAA Southeastern Pennsylvania Qualifier V1 0123")
////check speed fixed
////two intake motors
//// new bot
//
//public class TeleOpPIDFLearning extends LinearOpMode {
//    // 已有的硬件和常量定义...
//    private static final double VELOCITY_TOLERANCE = 30; // RPM容差，可根据测试调整
//    // 状态变量
//    private boolean isShooterAtSpeed = false;
//    private boolean wasShooterAtSpeed = false; // 用于检测状态变化
//    private boolean fireRequested = false;
//
//    private static final double Med_SHOOTER_TARGET_RPM = 1150;  // from 1200-1150 1666 still big 1866 kind of good for far， but a little bit too big
//
//
//    public float DriveTrains_ReducePOWER=1f;
//    public float DriveTrains_smoothTurn=1f;
//    HardwareQualifier robot = new HardwareQualifier();
////    public String fieldOrRobotCentric = "robot";
//    public String fieldOrRobotCentric = "field";
//    private double powerMultiplier = 0.9;
//    boolean move = false;
//    int controlMode = 1;
//    public float  intakePowerIntake=0.9f;//push blocker too much from 99-90
//    public float  intakePowerShoot=0.8f;
//    public float  intakePowerDump=-0.6f;
//    public float  intakePowerOff=0.0f;
//    public float  ShooterMotorShootFar=0.95f;
//    public float  ShooterMotorShootMed=-0.8f;
//    public float  ShooterMotorShootClose=-0.8f;
//    public float  ShooterMotorHold=-0.2f;
//    public float  ShooterMotorClean=-0.8f;
//    public float  ShooterMotorOff=0.0f;
//
//    private static final long IMU_RESET_COOLDOWN_MS = 300; // 1秒冷却时间
//
//    ElapsedTime delayTimer = new ElapsedTime();
//    // 计时器
//    private ElapsedTime runtime = new ElapsedTime();
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        FtcDashboard Dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = Dashboard.getTelemetry();
//        // 设置 Dashboard 更新频率
//        Dashboard.setTelemetryTransmissionInterval(100); // 100ms 更新一次
//
//        waitForStart();
//        runtime.reset();
//        delayTimer.reset();
//
//        while (opModeIsActive()) {
//
//            // 1. 更新底盘驱动
//            updateDrivetrain_FieldCentric();
//
//            // 2. 更新拾取系统
//            updateIntake();
//
//            // 4. 更新所有遥测数据（重要！）
//            telemetry.update();
//            // 5. 添加短暂延迟避免过于频繁的更新
//            sleep(20);
//
//        } //end of while loop
//
//    } //end of run mode
//
//
//    public void updateIntake() {
//            // 手柄控制拾取电机
//            if (gamepad1.left_trigger > 0.1) {
//                // 吸入
//                robot.IntakeMotorL.setPower(intakePowerIntake);
//                robot.IntakeMotorR.setPower(intakePowerIntake);
//
//                delayTimer.reset();
//                while (delayTimer.milliseconds() < 300 && opModeIsActive()) {
//                    // Other tasks can be processed here
//                } // 防止快速连击导致模式快速切换
//
//            } else if (gamepad1.left_bumper) {
//                // 反转（吐出）
//                robot.IntakeMotorL.setPower(intakePowerDump);
//                robot.IntakeMotorR.setPower(intakePowerDump);
//
//                delayTimer.reset();
//                while (delayTimer.milliseconds() < 300 && opModeIsActive()) {
//                    // Other tasks can be processed here
//                } // 防止快速连击导致模式快速切换
//
//            } else if (gamepad1.y)  {
//                // 停止
//                stopIntake();
//
//
//            }
//    }
//
//    public void updateShooter() {
//
//        // 手柄控制发射电机 - 按下右肩键启动射击电机
//        if (gamepad1.right_trigger > 0.1) {
//            if (!robot.MasterShooterMotorL.isBusy()){
//                startShooter();
//            }
//
//        }
//        // 手动射击触发 - 按下A键射击（仅在速度达标时有效）
////        if (gamepad1.right_bumper && isShooterAtSpeed && !fireRequested)
//        if (gamepad1.right_bumper) {
//            fireRequested = true;
//            robot.IntakeMotorL.setPower(intakePowerShoot);
//            robot.IntakeMotorR.setPower(intakePowerShoot);
//        }
//        // 停止射击电机 - 按下B键
//        if (gamepad1.b) {
//            stopShooter();
//            robot.IntakeMotorL.setPower(intakePowerOff);
//            robot.IntakeMotorR.setPower(intakePowerOff);
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
//
//    public static class ShooterPIDFConfig {
//        public static double kP = 100;     // 比例增益0.10.350.651.0655
//        public static double kI = 0.0;      // 积分增益
//        public static double kD = 0.0;      // 微分增益
//        public static double kF = 17;      // 前馈增益0.050.08
//        public static double targetRPM =Med_SHOOTER_TARGET_RPM; // 目标转速
//        public static double tolerance = 30;
//    }
//
//    private void initShooterPIDF() {
//
//    }
//
//    private void startShooter() {
//        robot.IntakeMotorL.setPower(0);
//        robot.IntakeMotorR.setPower(0);
//        if (robot.MasterShooterMotorL instanceof DcMotorEx) {
//            DcMotorEx shooter = (DcMotorEx) robot.MasterShooterMotorL;
////
//
//            //
//             //
//             //
//             //
//
//        }
//    }
//
//    private void updateShooterTelemetry() {
//        // 射击电机状态
//        double shooterVelocity = robot.MasterShooterMotorL.getVelocity();
//        double shooterPower = robot.MasterShooterMotorL.getPower();
//        double shooterCurrent = robot.MasterShooterMotorL.getCurrent(CurrentUnit.AMPS); // 如果有电流传感器
//        double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());
//        double targetVelocity = ShooterPIDFConfig.targetRPM;
//        double tolerance = ShooterPIDFConfig.tolerance;
//        telemetry.addLine("=== SHOOTER PIDF TUNING ===");
//        telemetry.addData("Target RPM", "%.0f", ShooterPIDFConfig.targetRPM);
//        telemetry.addData("Current RPM", "%.0f", shooterVelocity);
//        telemetry.addData("currentVelocity", currentVelocity);
//        telemetry.addData("targetVelocity", targetVelocity);
//        telemetry.addData("Error", "%.0f RPM", Math.abs(ShooterPIDFConfig.targetRPM - shooterVelocity));
//        telemetry.addData("At Speed?", isShooterAtSpeed ? "YES" : "NO");
//        telemetry.addData("Power", "%.2f", shooterPower);
//        // PIDF 参数值
//        telemetry.addLine("=== PIDF PARAMETERS ===");
//        telemetry.addData("kP", "%.4f", ShooterPIDFConfig.kP);
////        telemetry.addData("kI", "%.4f", ShooterPIDFConfig.kI);
////        telemetry.addData("kD", "%.4f", ShooterPIDFConfig.kD);
//        telemetry.addData("kF", "%.4f", ShooterPIDFConfig.kF);
//        telemetry.addData("Tolerance", "%.0f RPM", ShooterPIDFConfig.tolerance);
//
//        // 从电机状态
//        telemetry.addLine("=== SLAVE MOTOR ===");
//        telemetry.addData("Slave Power", "%.2f", robot.SlaveShooterMotorR.getPower());
//        telemetry.addData("Slave RPM", "%.0f", robot.SlaveShooterMotorR.getVelocity());
////
////        // 射击状态
////        telemetry.addLine("=== SHOOTING STATUS ===");
////        telemetry.addData("Fire Requested", fireRequested ? "YES" : "NO");
////        telemetry.addData("LED Color", isShooterAtSpeed ? "GREEN" : "RED");
//    }
//
//
//    /**
//     * 停止射击电机
//     */
//    private void stopShooter() {
//        robot.MasterShooterMotorL.setVelocity(0);
//        robot.SlaveShooterMotorR.setPower(0);
//        robot.IntakeMotorL.setPower(0);
//        robot. IntakeMotorR.setPower(0);
//        isShooterAtSpeed = false;
//        fireRequested = false;
//    }
//
//
//    ///  ///////////
//
//    private void stopIntake() {
//        robot.IntakeMotorL.setPower(intakePowerOff);
//        robot.IntakeMotorR.setPower(intakePowerOff);
//        robot.MasterShooterMotorL.setPower(ShooterMotorOff);
//        robot.SlaveShooterMotorR.setPower(ShooterMotorOff);
//        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // Non-blocking delay to prevent rapid mode switching
//        telemetry.addData("intakePowerOff", intakePowerOff);
//        telemetry.update();
//        delayTimer.reset();
//    }
//
//
//
//    /// ///////////
//
//    /**
//     * 执行射击序列
//     */
//     private double calculateOptimalSlavePower(double masterPower) {
//
//            double masterSpeed = robot.MasterShooterMotorL.getVelocity();
//            double slaveSpeed = robot.SlaveShooterMotorR.getVelocity();
//            double speedRatio = slaveSpeed / (masterSpeed + 0.001); // 避免除零
//
//            // 如果从电机速度明显落后，增加功率补偿
//            if (speedRatio < 0.9) {
//                return masterPower * 1.1;
//            }
//            // 如果从电机速度明显超前，减少功率
//            else if (speedRatio > 1.1) {
//                return masterPower * 0.9;
//            }
//            // 正常范围内使用相同功率
//            else {
//                return masterPower;
//            }
//
//    }
//    public void updateDrivetrain_FieldCentric() {
//            double y = gamepad1.left_stick_y * (1); // Remember, Y stick value is reversed
//            double x = -gamepad1.left_stick_x * (1);
//            double rx = -gamepad1.right_stick_x * DriveTrains_smoothTurn; //*(0.5) is fine
//
//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//// ******************************************temp
////        if (gamepad1.back) {
////            robot.imu.resetYaw();
////        }
////******************************************temp
//
//            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            robot.leftFrontMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
//            robot.leftRearMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
//            robot.rightFrontMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
//            robot.rightRearMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
//        }
//
//        public void updateDrivetrain_RobotCentric() {
//            double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double robot_x = gamepad1.left_stick_x;
//            double robot_rx = gamepad1.right_stick_x*DriveTrains_smoothTurn; // If a smooth turn is required 0.5
//
//            double fl = robot_y - robot_x - robot_rx;
//            double bl = robot_y + robot_x - robot_rx;
//            double fr = robot_y + robot_x + robot_rx;
//            double br = robot_y - robot_x + robot_rx;
//
//            robot.leftFrontMotor.setPower(fl * DriveTrains_ReducePOWER);
//            robot.leftRearMotor.setPower(bl * DriveTrains_ReducePOWER);
//            robot.rightFrontMotor.setPower(fr * DriveTrains_ReducePOWER);
//            robot.rightRearMotor.setPower(br * DriveTrains_ReducePOWER);
//
//        }
//
//
// }
//
//
//
//
//
//
//
//
