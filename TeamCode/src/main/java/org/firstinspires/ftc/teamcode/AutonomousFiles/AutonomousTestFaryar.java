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
@Autonomous(name = "autotestjohn", group = "Autonomous")
public class AutonomousTestFaryar extends LinearOpMode {
    @Override
    public void runOpMode() {
                waitForStart();
        Pose2d initialPose = new Pose2d(0, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Action traj = drive.actionBuilder(drive.pose)
// then your movements
// for example
                .strafeTo(new Vector2d(0,-40))
                .strafeToLinearHeading(new Vector2d(-48, -40), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-48, -46), Math.toRadians(45))
// and then at the end
                .build();

// finally, after waitForStart, run it:
        Actions.runBlocking(traj);



    }
}