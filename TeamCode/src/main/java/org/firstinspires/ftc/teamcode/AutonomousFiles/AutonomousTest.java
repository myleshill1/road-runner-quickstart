package org.firstinspires.ftc.teamcode.AutonomousFiles;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
            lift1.setDirection(DcMotorSimple.Direction.REVERSE);
            lift2 = hardwareMap.get(DcMotorEx.class, "slidesscoring2");
            lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift2.setDirection(DcMotorSimple.Direction.FORWARD);
            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                if (pos1 < 2000.0 && pos2 < 2000.0) {
                    telemetry.addData("lift1pos", pos1);
                    telemetry.addData("lift2pos", pos2);
                    telemetry.update();
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
        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                packet.put("lift2Pos", pos2);
                if (pos1 > 0.0 && pos2 < 0.0) {
                    telemetry.addData("lift1pos", pos1);
                    telemetry.addData("lift2pos", pos2);
                    telemetry.update();
                    return true;
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }

    public class Pivot {
        private DcMotorEx pivot1;
        private DcMotorEx pivot2;

        public Pivot(HardwareMap hardwareMap) {
            pivot1 = hardwareMap.get(DcMotorEx.class, "pivot");
            pivot1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            pivot1.setDirection(DcMotorSimple.Direction.REVERSE);
            pivot2 = hardwareMap.get(DcMotorEx.class, "pivot2");
            pivot2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            pivot2.setDirection(DcMotorSimple.Direction.FORWARD);
            pivot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class PivotUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot1.setPower(1);
                    pivot2.setPower(1);
                    initialized = true;
                }

                double pos1 = pivot1.getCurrentPosition();
                double pos2 = pivot2.getCurrentPosition();
                packet.put("pivot1pos", pos1);
                packet.put("pivot2pos", pos2);
                if (pos1 < 1200.0 && pos2 < 1200.0) {
                    telemetry.addData("pivot1pos", pos1);
                    telemetry.addData("pivot2pos", pos2);
                    telemetry.update();
                    return true;
                } else {
                    pivot1.setPower(0);
                    pivot2.setPower(0);
                    return false;
                }
            }
        }

        public Action pivotUp() {
            return new PivotUp();
        }
    }


        @Override
    public void runOpMode() {

                waitForStart();
        Pose2d initialPose = new Pose2d(0, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //put initialization here
        Actions.runBlocking(lift.liftDown());
        //init ends

        //
        Action liftAction1 = lift.liftUp();
        Action pivotAction1 = pivot.pivotUp();
        Action trajectory = drive.actionBuilder(drive.pose)
// then your movements
// for example
                .strafeTo(new Vector2d(0,-40))
              //  .strafeToLinearHeading(new Vector2d(-48, -40), Math.toRadians(90))
               // .strafeToLinearHeading(new Vector2d(-48, -46), Math.toRadians(45))
// and then at the end
                .build();

// finally, after waitForStart, run it:
        Actions.runBlocking(trajectory);
        Actions.runBlocking(pivotAction1);
        Actions.runBlocking(liftAction1);



    }
}
