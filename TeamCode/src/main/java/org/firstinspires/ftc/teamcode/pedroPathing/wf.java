//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@TeleOp(name="XDrive Single File")
//public class wf extends LinearOpMode {
//
//    private DcMotor frontLeft;
//    private DcMotor backLeft;
//    private DcMotor frontRight;
//    private DcMotor backRight;
//
//    @Override
//    public void runOpMode() {
//        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
//        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
//        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
//        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");
//
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            double forward = -gamepad1.left_stick_y;
//            double strafe  = gamepad1.left_stick_x;
//            double rotate  = gamepad1.right_stick_x;
//
//
//            double frontLeftPower  = forward + strafe + rotate;
//            double backLeftPower   = forward - strafe + rotate;
//            double frontRightPower = forward - strafe - rotate;
//            double backRightPower  = forward + strafe - rotate;
//
//            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
//                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
//
//            if (max > 1.0) {
//                frontLeftPower  /= max;
//                backLeftPower   /= max;
//                frontRightPower /= max;
//                backRightPower  /= max;
//            }
//
//            frontLeft.setPower(frontLeftPower);
//            backLeft.setPower(backLeftPower);
//            frontRight.setPower(frontRightPower);
//            backRight.setPower(backRightPower);
//        }
//    }
//}