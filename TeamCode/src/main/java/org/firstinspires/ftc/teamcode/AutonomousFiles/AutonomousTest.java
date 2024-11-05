package org.firstinspires.ftc.teamcode.AutonomousFiles;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Autonomous(name = "autotest1", group = "Autonomous")
public class AutonomousTest extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift1;
        private DcMotorEx lift2;

        public Lift(HardwareMap hardwareMap) {
            lift1 = hardwareMap.get(DcMotorEx.class, "scoringslides");
            lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift1.setDirection(DcMotorSimple.Direction.FORWARD);
            lift2 = hardwareMap.get(DcMotorEx.class, "slidesscoring2");
            lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(1);
                    lift2.setPower(1);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                packet.put("lift2Pos", pos2);
                if (pos1 < 1300.0 && pos2 < 1300.0) {
                    return true;
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
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
        Lift lift = new Lift(hardwareMap);

        Action liftAction1 = lift.liftUp();
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
        Actions.runBlocking(liftAction1);


    }
}
