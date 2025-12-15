package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HardwareGobilda {
    HardwareMap hwMap =  null;

    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightRearMotor;
    public DcMotorEx leftRearMotor;


    IMU imu;
    public static final double DriveTrains_POWER =  0.95 ;// reduced power of driving train motors

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        leftFrontMotor   = hwMap.get(DcMotorEx.class, "LFMotor");//11072025 control hub port 2
        rightFrontMotor  = hwMap.get(DcMotorEx.class, "RFMotor"); //11072025 expansion hub port 0
        leftRearMotor   = hwMap.get(DcMotorEx.class, "LBMotor");//11072025 control hub port 1
        rightRearMotor  = hwMap.get(DcMotorEx.class, "RBMotor"); //11072025 expansion hub port 1

        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        setAllPower(0);



        ///////////////////////////////////////GoBildaPinpointDriver/////////////////////////////


        imu = hwMap.get(IMU.class, "imu");  //control I2C port 1
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        imu.initialize(parameters);


    }
    public void setMotorPower(double lF, double rF, double lB, double rB){
        leftFrontMotor.setPower(lF*DriveTrains_POWER);
        leftRearMotor.setPower(lB*DriveTrains_POWER);
        rightRearMotor.setPower(rB*DriveTrains_POWER);
        rightFrontMotor.setPower(rF*DriveTrains_POWER);
    }
    public void setAllPower(double p){
        setMotorPower(p,p,p,p);
    }

}
