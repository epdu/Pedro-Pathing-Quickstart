//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class rr {
//
//    private DcMotor frontLeft;
//    private DcMotor backLeft;
//    private DcMotor frontRight;
//    private DcMotor backRight;
//
//    public void init(HardwareMap hardwareMap) {
//
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
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    public void drive(double forward, double strafe, double rotate) {
//
//        double frontLeftPower  = forward + strafe + rotate;
//        double backLeftPower   = forward - strafe + rotate;
//        double frontRightPower = forward - strafe - rotate;
//        double backRightPower  = forward + strafe - rotate;
//
//        double max = Math.max(
//                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
//                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
//        );
//
//        if (max > 1.0) {
//            frontLeftPower  /= max;
//            backLeftPower   /= max;
//            frontRightPower /= max;
//            backRightPower  /= max;
//        }
//
//        frontLeft.setPower(frontLeftPower);
//        backLeft.setPower(backLeftPower);
//        frontRight.setPower(frontRightPower);
//        backRight.setPower(backRightPower);
//    }
//
//    public void stop() {
//        frontLeft.setPower(0);
//        backLeft.setPower(0);
//        frontRight.setPower(0);
//        backRight.setPower(0);
//    }
//}