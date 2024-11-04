package org.firstinspires.ftc.teamcode.AutonomousFiles;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "autotestcoloredside", group = "Autonomous")
public class AutonomousColoredSide extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        Pose2d initialPose = new Pose2d(0, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Action poop = drive.actionBuilder(drive.pose)
// then your movement
// for example
                .strafeToLinearHeading(new Vector2d(0,-40), Math.toRadians(90))
                //score specimen w claw
                //.strafeToLinearHeading(new Vector2d(36, -40), Math.toRadians(180))
                .setTangent(180)
                .strafeToLinearHeading(new Vector2d(36, -10), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(46, -10), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(46, -50), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(46, -10), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(56, -10), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(56, -50), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(56, -10), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(61, -10), Math.toRadians(180))
                //.strafeToLinearHeading(new Vector2d(61, -50), Math.toRadians(180))

// and then at the end
                .build();

// finally, after waitForStart, run it:
        Actions.runBlocking(poop);



    }
}