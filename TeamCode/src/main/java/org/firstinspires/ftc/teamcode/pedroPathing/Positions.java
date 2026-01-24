
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Positions {
    public static double tileLength = 23.5;


    /*** Start Positions ***/
    public static Pose startFar = new Pose(48, 9, Math.toRadians(90));
    public static Pose startClose = new Pose(23, 126, Math.toRadians(135));

    /*** Launch Positions ***/
    public static Pose launchFar = new Pose(50, 12, Math.toRadians(110));
    public static Pose launchClose = new Pose(2.5*tileLength, 144-2.5*tileLength, Math.toRadians(135));
    public static Pose launchOutOfTheWay = new Pose(2.5*tileLength, 144-1.5*tileLength, Math.toRadians(145));


    /*** Load Positions ***/
    public static Pose loadingZoneStart = new Pose(tileLength, tileLength/2, Math.toRadians(180));
    public static Pose loadingZoneEnd = new Pose(9, tileLength/2, Math.toRadians(180));
    public static Pose stack1Start = new Pose(40, 1.5*tileLength, Math.toRadians(180));
    public static Pose stack1End = new Pose(15, 1.5*tileLength, Math.toRadians(180));
    public static Pose stack2Start = new Pose(40, 2.5*tileLength, Math.toRadians(180));
    public static Pose stack2End = new Pose(15, 2.5*tileLength, Math.toRadians(180));
    public static Pose stack3Start = new Pose(40, 3.5*tileLength, Math.toRadians(180));
    public static Pose stack3End = new Pose(15, 3.5*tileLength, Math.toRadians(180));


    /*** Park Positions ***/
    public static Pose parkFar = new Pose(1.5*tileLength, tileLength/2, Math.toRadians(0));
    public static Pose parkClose = new Pose(2.5*tileLength, 144-9, Math.toRadians(0));
    public static Pose parkMid = new Pose(2.5*tileLength, 2*tileLength, Math.toRadians(0));

    /**** Targets *****/


    public static double redTargetX=140;
    public static double redTargetY=140;
    public static double blueTargetX=4;
    public static double blueTargetY=140;

}
