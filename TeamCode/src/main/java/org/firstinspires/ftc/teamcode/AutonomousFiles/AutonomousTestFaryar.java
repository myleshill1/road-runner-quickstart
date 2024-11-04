package org.firstinspires.ftc.teamcode.AutonomousFiles;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Autonomous(name = "autotestjohn", group = "Autonomous")
public class AutonomousTestFaryar extends LinearOpMode {
    private static class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "scoringslides");
            lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorEx.Direction.FORWARD);
            lift = hardwareMap.get(DcMotorEx.class, "slidesscoring2");
            lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorEx.Direction.FORWARD);
        }
        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(1);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1300.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

    }

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
                .waitSeconds(1)
                .turnTo(Math.toRadians(120))
                .turnTo(Math.toRadians(45))
                .turnTo(Math.toRadians(133))
                .turnTo(Math.toRadians(45))
                .waitSeconds(1)
                .turn(Math.toRadians(85.5))
                .splineToLinearHeading(new Pose2d(-26, -10, Math.toRadians(0)), Math.toRadians(25))


                //.turnTo(Math.toRadians(90))
                //.strafeTo(new Vector2d(-48, 3))
                //.turnTo(Math.toRadians(0))
                //.strafeTo(new Vector2d(-26.83,3))
// and then at the end
                .build();

// finally, after waitForStart, run it:
        Actions.runBlocking(traj);



    }
}