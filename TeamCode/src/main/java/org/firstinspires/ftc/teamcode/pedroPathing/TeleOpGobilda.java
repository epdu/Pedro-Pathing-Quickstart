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
@TeleOp(name = "Gobilda V1 1205")
//V1 with pid for xxx  but not odo
public class TeleOpGobilda extends LinearOpMode {
    // 已有的硬件和常量定义...

    public float DriveTrains_ReducePOWER=0.75f;
    public float DriveTrains_smoothTurn=0.55f;
    HardwareGobilda robot = new HardwareGobilda();
    public String fieldOrRobotCentric = "robot";
//    public String fieldOrRobotCentric = "field";
    private double powerMultiplier = 0.9;
    boolean move = false;
    int controlMode = 1;

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

            // 4. 更新所有遥测数据（重要！）
            telemetry.update();
            // 5. 添加短暂延迟避免过于频繁的更新
            sleep(20);

        } //end of while loop

    } //end of run mode

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







 }








