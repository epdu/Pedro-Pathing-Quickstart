package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class wrf extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Declare and Initialize Motors with full names
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // 2. Initialize the IMU (Internal Gyro)
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        waitForStart();

        while (opModeIsActive()) {
            // 3. Read joystick inputs
            double y = -gamepad1.left_stick_y; // Reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Reset heading to zero when 'Options' is pressed
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // 4. Calculate robot heading and rotate the inputs
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Calculate the rotated X and Y (Field-Centric)
            // The 1.1 multiplier accounts for mecanum strafe friction
            double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // 5. Normalize motor powers so no value exceeds 1.0
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // 6. Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}