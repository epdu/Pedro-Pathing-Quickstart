//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.HardwareQualifier;
//
//@Autonomous
//public class autoleave extends LinearOpMode {
//
//    public float DriveTrains_ReducePOWER = 0.7f;
//    HardwareQualifier robot = new HardwareQualifier();
//
//    public String fieldOrRobotCentric = "robot";
//    boolean move = false;
//    boolean movementActive = false;
//
//    private volatile boolean isRunning = true;
//    ElapsedTime delayTimer = new ElapsedTime();
//
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
////        gyro.robot.init(hardwareMap);
//
//        waitForStart();
//
//        robot.leftRearMotor.setPower(-0.6);
//        robot.leftFrontMotor.setPower(-0.6);
//        robot.rightRearMotor.setPower(-0.6);
//        robot.rightFrontMotor.setPower(-0.6);
//        sleep(200);
//        robot.leftRearMotor.setPower(0);
//        robot.leftFrontMotor.setPower(0);
//        robot.rightRearMotor.setPower(0);
//        robot.rightFrontMotor.setPower(0);
//
//        stop();
//    }
//}
