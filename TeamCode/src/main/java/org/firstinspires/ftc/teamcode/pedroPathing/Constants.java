package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.117);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.8)
            .rightFrontMotorName("RFMotor")
            .rightRearMotorName("RBMotor")
            .leftRearMotorName("LBMotor")
            .leftFrontMotorName("LFMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(78.401);
//public static MecanumConstants driveConstants = new MecanumConstants()
//        .maxPower(0.8)
//        .rightFrontMotorName("RFMotor")
//        .rightRearMotorName("RBMotor")
//        .leftRearMotorName("LBMotor")
//        .leftFrontMotorName("LFMotor")
//        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//        .xVelocity(78.401);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-1.9685)
            .strafePodX(-7.08661)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}
