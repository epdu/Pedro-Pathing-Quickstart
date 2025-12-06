package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config  // 添加这个注解，让 Dashboard 可以调整参数
@TeleOp(name = "A Blue Streak Qualifier V1")
//V1 with pid for xxx  but not odo
public class TeleOpQualifier extends LinearOpMode {
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

    // RPM = (TPS * 60秒) / 每转ticks数
//    return (tps * 60.0) / ticksPerRevolution;  28*13.7
    private static final double Close_SHOOTER_TARGET_RPM = 2557;//  400RPM---2,557.33333333333333
    private static final double Med_SHOOTER_TARGET_RPM = 1598;   ////  250RPM---1586.67
    private static final double Far_SHOOTER_TARGET_RPM = 2237;  //  350RPM---2237
    //  1000RPM---6346.67
    //  600RPM---3808
    //  500RPM---3173.3
    public float DriveTrains_ReducePOWER=0.75f;
    public float DriveTrains_smoothTurn=0.55f;
    HardwareQualifier robot = new HardwareQualifier();
    public String fieldOrRobotCentric = "robot";
//    public String fieldOrRobotCentric = "field";
    private double powerMultiplier = 0.9;
    boolean move = false;
    int controlMode = 1;
    public float  intakePowerIntake=0.85f;
    public float  intakePowerShoot=0.4f;
    public float  intakePowerDump=-0.6f;
    public float  intakePowerOff=0.0f;
    public float  ShooterMotorShootFar=0.95f;
    public float  ShooterMotorShootMed=-0.8f;
    public float  ShooterMotorShootClose=-0.8f;
    public float  ShooterMotorHold=-0.2f;
    public float  ShooterMotorClean=-0.8f;
    public float  ShooterMotorOff=0.0f;
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
        initShooterPIDF();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard Dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = Dashboard.getTelemetry();
        // 设置 Dashboard 更新频率
        Dashboard.setTelemetryTransmissionInterval(100); // 100ms 更新一次

        waitForStart();
        runtime.reset();
        delayTimer.reset();

        while (opModeIsActive()) {

            // 1. 更新底盘驱动
            updateDrivetrain_FieldCentric();
//            updateDrivetrain_RobotCentric();
            // 2. 更新拾取系统
            updateIntake();
            // 3. 更新射击系统
            updateShooter();
            // 4. 更新所有遥测数据（重要！）
            telemetry.update();
            // 5. 添加短暂延迟避免过于频繁的更新
            sleep(20);

        } //end of while loop

    } //end of run mode


    public void updateIntake() {
            // 手柄控制拾取电机
            if (gamepad1.left_trigger > 0.1) {
                // 吸入
                robot.IntakeMotor.setPower(intakePowerIntake);
                robot.MasterShooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
//                robot.MasterShooterMotorL.setPower(ShooterMotorHold);
//                robot.SlaveShooterMotorR.setPower(ShooterMotorHold);
                // Non-blocking delay to prevent rapid mode switching

                telemetry.addData("intakePowerDump", intakePowerDump);
                telemetry.update();
                delayTimer.reset();
                while (delayTimer.milliseconds() < 300 && opModeIsActive()) {
                    // Other tasks can be processed here
                } // 防止快速连击导致模式快速切换

            } else {
                // 停止
                robot.IntakeMotor.setPower(intakePowerOff);
                robot.MasterShooterMotorL.setPower(ShooterMotorOff);
                robot.SlaveShooterMotorR.setPower(ShooterMotorOff);
                // Non-blocking delay to prevent rapid mode switching
                telemetry.addData("intakePowerOff", intakePowerOff);
                telemetry.update();
                delayTimer.reset();

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
        if (gamepad1.right_bumper && isShooterAtSpeed && !fireRequested) {
            fireRequested = true;
            executeFireSequence();
        }
        // 停止射击电机 - 按下B键
        if (gamepad1.b) {
            stopShooter();
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
                    ShooterPIDFConfig.kP,
                    ShooterPIDFConfig.kI,
                    ShooterPIDFConfig.kD,
                    ShooterPIDFConfig.kF
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
            shooter.setVelocity(ShooterPIDFConfig.targetRPM);

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

    /**
     * 检查射击电机速度（使用 Dashboard 调整的容差）
     */
    private void checkShooterVelocity() {
        if (robot.MasterShooterMotorL.isBusy()) {
            double currentVelocity = Math.abs(robot.MasterShooterMotorL.getVelocity());
            double targetVelocity = ShooterPIDFConfig.targetRPM;
            double tolerance = ShooterPIDFConfig.tolerance;

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
    }


    /**
     * 执行射击序列
     */
    private void executeFireSequence() {
        if (!isShooterAtSpeed) return; // 安全检查
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

            robot.LFMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
            robot.LBMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
            robot.RFMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
            robot.RBMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
        }

        public void updateDrivetrain_RobotCentric() {
            double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double robot_x = gamepad1.left_stick_x;
            double robot_rx = gamepad1.right_stick_x*DriveTrains_smoothTurn; // If a smooth turn is required 0.5

            double fl = robot_y - robot_x - robot_rx;
            double bl = robot_y + robot_x - robot_rx;
            double fr = robot_y + robot_x + robot_rx;
            double br = robot_y - robot_x + robot_rx;

            robot.LFMotor.setPower(fl * DriveTrains_ReducePOWER);
            robot.LBMotor.setPower(bl * DriveTrains_ReducePOWER);
            robot.RFMotor.setPower(fr * DriveTrains_ReducePOWER);
            robot.RBMotor.setPower(br * DriveTrains_ReducePOWER);

        }

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








