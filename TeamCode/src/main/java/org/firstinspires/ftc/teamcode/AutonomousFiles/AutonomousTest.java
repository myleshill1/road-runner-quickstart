package org.firstinspires.ftc.teamcode.AutonomousFiles;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "Claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(-0.5);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }


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
                    lift1.setPower(0.5);
                    lift2.setPower(0.5);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                packet.put("lift2Pos", pos2);
                if (pos1 <= 1450.0 && pos2 <= 1450.0) {
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

        public class Highbasketlift implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(0.5);
                    lift2.setPower(0.5);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                packet.put("lift2Pos", pos2);
                if (pos1 <= 3300.0 && pos2 <= 3300.0) {
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

        public Action highbasketLift() {
            return new Highbasketlift();
        }

        public class Highbasketlift2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(0.5);
                    lift2.setPower(0.5);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                packet.put("lift2Pos", pos2);
                if (pos1 <= 3300.0 && pos2 <= 3300.0) {
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

        public Action highbasketLift2() {
            return new Highbasketlift2();
        }


        public class LiftSample implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(0.5);
                    lift2.setPower(0.5);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                packet.put("lift2Pos", pos2);
                if (pos1 <= 950.0 && pos2 <= 950.0) {
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

        public Action liftSample() {
            return new LiftSample();
        }

        public class LiftSamplefromhigh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(-0.5);
                    lift2.setPower(-0.5);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                packet.put("lift2Pos", pos2);
                if (pos1 >= 890.0 && pos2 >= 890.0) {
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
        public Action liftSamplefromhigh() {
            return new LiftSamplefromhigh();
        }
        public class LiftSamplefromhigh2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(-0.5);
                    lift2.setPower(-0.5);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                packet.put("lift2Pos", pos2);
                if (pos1 >= 830.0 && pos2 >= 830.0) {
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

        public Action liftSamplefromhigh2() {
            return new LiftSamplefromhigh2();
        }


        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(-0.5);
                    lift2.setPower(-0.5);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                packet.put("lift2Pos", pos2);
                if (pos1 >= 0.0 && pos2 >= 0.0) {
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
                    pivot1.setPower(0.3);
                    pivot2.setPower(0.3);
                    initialized = true;
                }

                double pos1 = pivot1.getCurrentPosition();
                double pos2 = pivot2.getCurrentPosition();
                packet.put("pivot1pos", pos1);
                packet.put("pivot2pos", pos2);
                if (pos1 <= 1293.0 && pos2 <= 1293.0) {
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
/*penis*/
        public Action pivotUp() {
            return new PivotUp();
        }
        public class pivotdowntosample implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot1.setPower(-0.5);
                    pivot2.setPower(-0.5);
                    initialized = true;
                }

                double pos1 = pivot1.getCurrentPosition();
                double pos2 = pivot2.getCurrentPosition();
                packet.put("pivot1pos", pos1);
                packet.put("pivot2pos", pos2);
                if (pos1 >= 90.0 && pos2 >= 90.0) {
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
        public Action pivotdowntosample() {
            return new pivotdowntosample();
        }

        public class pivotdowntosample2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot1.setPower(-0.5);
                    pivot2.setPower(-0.5);
                    initialized = true;
                }

                double pos1 = pivot1.getCurrentPosition();
                double pos2 = pivot2.getCurrentPosition();
                packet.put("pivot1pos", pos1);
                packet.put("pivot2pos", pos2);
                if (pos1 >= 80.0 && pos2 >= 80.0) {
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
        public Action pivotdowntosample2() {
            return new pivotdowntosample2();
        }

        public class pivotdowntosample3 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot1.setPower(-0.5);
                    pivot2.setPower(-0.5);
                    initialized = true;
                }

                double pos1 = pivot1.getCurrentPosition();
                double pos2 = pivot2.getCurrentPosition();
                packet.put("pivot1pos", pos1);
                packet.put("pivot2pos", pos2);
                if (pos1 >= 105.0 && pos2 >= 105.0) {
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
        public Action pivotdowntosample3() {
            return new pivotdowntosample3();
        }
        public class Pivotdown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot1.setPower(-0.5);
                    pivot2.setPower(-0.5);
                    initialized = true;
                }

                double pos1 = pivot1.getCurrentPosition();
                double pos2 = pivot2.getCurrentPosition();
                packet.put("pivot1pos", pos1);
                packet.put("pivot2pos", pos2);
                if (pos1 >= 0.0 && pos2 >= 0.0) {
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
        public Action pivotDown() {
            return new Pivotdown();
        }

        public class Pivothighbasket implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot1.setPower(0.3);
                    pivot2.setPower(0.3);
                    initialized = true;
                }

                double pos1 = pivot1.getCurrentPosition();
                double pos2 = pivot2.getCurrentPosition();
                packet.put("pivot1pos", pos1);
                packet.put("pivot2pos", pos2);
                if (pos1 <= 1390 && pos2 <= 1390.0) {
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
        public Action pivothighbasket() {
            return new Pivothighbasket();
        }

        public class Pivothighbasket2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot1.setPower(0.3);
                    pivot2.setPower(0.3);
                    initialized = true;
                }

                double pos1 = pivot1.getCurrentPosition();
                double pos2 = pivot2.getCurrentPosition();
                packet.put("pivot1pos", pos1);
                packet.put("pivot2pos", pos2);
                if (pos1 <= 1390.0 && pos2 <= 1390.0) {
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
        public Action pivothighbasket2() {
            return new Pivothighbasket2();
        }

        public class Maxpivot1 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot1.setPower(0.3);
                    pivot2.setPower(0.3);
                    initialized = true;
                }

                double pos1 = pivot1.getCurrentPosition();
                double pos2 = pivot2.getCurrentPosition();
                packet.put("pivot1pos", pos1);
                packet.put("pivot2pos", pos2);
                if (pos1 <= 1500.0 && pos2 <= 1500.0) {
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
        public Action maxpivot1() {
            return new Maxpivot1();
        }

        public class Maxpivot2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot1.setPower(0.3);
                    pivot2.setPower(0.3);
                    initialized = true;
                }

                double pos1 = pivot1.getCurrentPosition();
                double pos2 = pivot2.getCurrentPosition();
                packet.put("pivot1pos", pos1);
                packet.put("pivot2pos", pos2);
                if (pos1 <= 1500.0 && pos2 <= 1500.0) {
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
        public Action maxpivot2() {
            return new Maxpivot2();
        }
    }


        @Override
    public void runOpMode() {



        Pose2d initialPose = new Pose2d(0, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //put initialization here
        //Actions.runBlocking(lift.liftDown());
        //init ends


        Action specimenLift = lift.liftUp();
        Action liftDown = lift.liftDown();
        Action sampleLiftfrom0 = lift.liftSample();
        Action sampleLiftfromhigh = lift.liftSamplefromhigh();
            Action sampleLiftfromhigh2 = lift.liftSamplefromhigh2();
        Action highbasketSlides = lift.highbasketLift();
        Action highbasketSlides2 = lift.highbasketLift2();

        Action openClaw = claw.openClaw();
        Action closeClaw = claw.closeClaw();

        Action specimenPivotUp = pivot.pivotUp();
        Action specimenpivotdowntosample1 = pivot.pivotdowntosample();
        Action specimenpivotdowntosample2 = pivot.pivotdowntosample2();
        Action specimenpivotdowntosample3 = pivot.pivotdowntosample3();

        Action pivotdown = pivot.pivotDown();
        Action highbasketPivotUp = pivot.pivothighbasket();
        Action highbasketPivotUp2 = pivot.pivothighbasket2();
        Action maxPivot1 = pivot.maxpivot1();
        Action maxPivot2 = pivot.maxpivot2();

        Action fullpath = drive.actionBuilder(initialPose)
                .waitSeconds(3)
                .strafeTo(new Vector2d(0,-31))
                .waitSeconds(1.5)

                .strafeTo(new Vector2d(0, -40))

                .waitSeconds(0.5)

                .strafeToLinearHeading(new Vector2d(-48.5, -40), Math.toRadians(90))

                //.setTangent(180)
                //.splineToLinearHeading(new Pose2d(-48.5, -40, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(4.5)
                //.strafeToLinearHeading(new Vector2d(-48, -46), Math.toRadians(45))
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(-57.5, -57.5, Math.toRadians(230)), Math.toRadians(230))
                //.strafeToLinearHeading(new Vector2d(-57.5, -57.5), Math.toRadians(230))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(230))
                //sample one high basket
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-56.5, -40), Math.toRadians(90))
                .waitSeconds(4)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-57.5, -57.5, Math.toRadians(230)), Math.toRadians(180))
                //.strafeToLinearHeading(new Vector2d(-57.5, -57.5), Math.toRadians(230))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(230))

               // .strafeToLinearHeading(new Vector2d(-56, -26), Math.toRadians(180))
                //.strafeToLinearHeading(new Vector2d(-57.5, -57.5), Math.toRadians(230))

                .splineToLinearHeading(new Pose2d(-26, -10, Math.toRadians(0)), Math.toRadians(-25))
                .build();
        Action specimenForward = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,-31))
              //  .strafeToLinearHeading(new Vector2d(-48, -40), Math.toRadians(90))
               // .strafeToLinearHeading(new Vector2d(-48, -46), Math.toRadians(45))
                .build();

        Action specimenToSample1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-48, -40), Math.toRadians(90))
                .build();
        Action specimenhalfForward = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0,-40.5), Math.toRadians(90))
                        .build();
        Action moverightone = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(1,0))
                .build();

        //put inits here
        //Actions.runBlocking(claw.closeClaw());
// finally, after waitForStart, run it:
waitForStart();
if (isStopRequested()) return;
// autonomous starts
Actions.runBlocking(
        new ParallelAction(
                fullpath,
                new SequentialAction(
                        closeClaw,
                        specimenPivotUp,
                        specimenLift,
                        new SleepAction(1.5),
                        new ParallelAction(
                                liftDown,
                                new SequentialAction(
                                        new SleepAction(0.825),
                                        openClaw,
                                        new SleepAction(1),
                                        maxPivot1
                                )
                        ),
                        //specimen clipped on
                        new SleepAction(1),
                        specimenpivotdowntosample1,
                        sampleLiftfrom0,
                        new SleepAction(0.5),
                        closeClaw,
                        new SleepAction(0.1),
                        new ParallelAction(
                                highbasketPivotUp,
                                new SequentialAction(
                                        new SleepAction(1),
                                        highbasketSlides

                                )
                        ),
                        new SleepAction(1.3),
                        openClaw,
                        //sample 1 basket done
                        new SleepAction(0.7),
                        maxPivot2,
                        sampleLiftfromhigh,
                        new SleepAction(2),
                        specimenpivotdowntosample2,
                        new SleepAction(0.5),
                        closeClaw,
                        new SleepAction(0.1),
                        new ParallelAction(
                                highbasketPivotUp2,
                                new SequentialAction(
                                        new SleepAction(1),

                                        highbasketSlides2
                                )

                        ),
                        new SleepAction(1),
                        openClaw,
                        //sample 2 basket done
                        new SleepAction(3),
                        new ParallelAction(
                                sampleLiftfromhigh2,
                                new SleepAction(2),
                                specimenpivotdowntosample3
                        )
                )
        )
);



    }
}
