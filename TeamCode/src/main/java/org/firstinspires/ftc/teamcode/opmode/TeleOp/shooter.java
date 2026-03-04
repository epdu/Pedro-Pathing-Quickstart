//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//public class shooter {
//
//
//    private Vector calculateshotVectorAndUpdateTurret(double robotHeading){
//
////constant
//        double g = 32.174 * 12;
//        double x= robotToGoalVector.getMagnitude()- shooterconstants.PASS_THROUGH_POINT_RADIUS;
//        double y= ShooterConstants.SCORE_HEIGHT;
//        double a= Shooterconstants.SCORE_ANGLE;
////calculate initial launch component
//        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x- Math.tan(a)), Shooterconstants.HO0D_MAX_ANGLEShooterConstants.HOOD_MIN_ANGLE);
//        double flywheelspeed = Math.sqrt(g * x * x / (2 * Hath.pow(Math.cos(hoodAngle), 2)* (x * Hath,tan(hoodAngle) - y)));
////get robot velocity and convert it into parallel and perpendicular components
//        Vector robotVelocity = hardware.poseTracker.getVelocity();
//        double coordinateTheta = robotVelocity,getTheta()- robotToGoalVector.getTheta();
//        double parallelcomponent = -Math,cos(coordinateTheta)* robotVelocity.getMagnitude();
//        double perpendicularComponent = Math.sin(coordinateTheta)* robotVelocity.getMagnitude();
////velocity compensation variablesdouble vz = flywheelSpeed * Hath.sin(hoodAngle);double time =x/(flywheelSpeed * Math.cos(hoodAngle));double ivr = x/ time + parallelcomponentdouble nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);double ndr = nvr * time;
//
////recalculate launch components
//        hoodAngle = MathFunctions.clamp(Math.atan(vz /nvr),ShooterConstants.HOOD_MAX_ANGLEShooterConstants.HO0D_MIN_ANGLE);
//        flywheelspeed = Math.sgrt(g * ndr * ndr / (2 * Hath.pow(Nath.cos(hoodAngle), 2)* (ndr * Hath.tan(hoodAngle)- y)));
////update turretdouble turretVelcompoffset = Math.atan(perpendicularComponent / ivr);
//
//}
