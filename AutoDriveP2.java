package org.firstinspires.ftc.teamcode;
import android.os.SystemClock;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
@Autonomous(name = "Full Auto Drive (P2)")
public class AutoDriveP2 extends LinearOpMode {
    public Servo wrist;
    public Servo intake;
    public class Lift {
        private DcMotorEx lift;
        private DcMotorEx arm;
        double chk1 = 0;
        double stage1 = 1;
        boolean chk2 = false;
        double stage2 = 1;
        double chk3 = 0;
        double stage3 = 1;
        double tt = 0;
        TouchSensor slideTouch = hardwareMap.touchSensor.get("slideTouch");
        TouchSensor armTouch = hardwareMap.touchSensor.get("armTouch");
        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "SliMot");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            arm = hardwareMap.get(DcMotorEx.class, "RotMot");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            wrist = hardwareMap.get(Servo.class, "wrist");
            intake = hardwareMap.get(Servo.class, "intake");
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public class LiftUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double slpos = lift.getCurrentPosition();
                double armpos = arm.getCurrentPosition();
                if (chk1<685 && chk1!=0){
                    telemetry.addLine("Ended liftUp action!");
                    telemetry.update();
                    return false;
                } else if (stage1==1){
                    telemetry.addLine("Stage1 liftUp action!");
                    telemetry.addLine("ArmPos: "+armpos);
                    telemetry.addLine("SlidePos: "+slpos);
                    telemetry.update();
                    arm.setTargetPosition(889);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.7);
                    wrist.setPosition(0.465);
                    intake.setPosition(0.5);
                    stage1 = 2;
                } else if (stage1==2){
                    if (armpos>800){
                        lift.setTargetPosition(1445);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                        telemetry.addLine("Stage2 liftUp action!");
                        telemetry.addLine("ArmPos: "+armpos);
                        telemetry.addLine("SlidePos: "+slpos);
                        telemetry.update();
                        wrist.setPosition(0);
                        intake.setPosition(1);
                        stage1 = 3;
                    }
                } else if (stage1==3){
                    if (slpos>1440) {
                        arm.setTargetPosition(680);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.8);
                        telemetry.addLine("Stage3 liftUp action!");
                        telemetry.addLine("ArmPos: "+armpos);
                        telemetry.addLine("SlidePos: "+slpos);
                        telemetry.update();
                        chk1=arm.getCurrentPosition();
                    }
                }
                return true;
            }
        }

        public class LiftDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double slpos = lift.getCurrentPosition();
                double armpos = arm.getCurrentPosition();

                if (chk2){
                    telemetry.addLine("Ended liftDown action!");
                    telemetry.update();
                    return false;
                } else if (stage2==1){;
                    telemetry.addLine("Stage1 liftDown action!");
                    telemetry.addLine("ArmPos: "+armpos);
                    telemetry.addLine("SlidePos: "+slpos);
                    telemetry.update();
                    lift.setTargetPosition(1);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                    stage2=2;
                } else if (stage2==2){
                    if (!slideTouch.isPressed() || lift.getCurrentPosition()<=2){
                        arm.setTargetPosition(1);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.7);
                        telemetry.addLine("Stage2 liftDown action!");
                        telemetry.addLine("ArmPos: "+armpos);
                        telemetry.addLine("SlidePos: "+slpos);
                        telemetry.update();
                        intake.setPosition(0.5);
                        wrist.setPosition(0.65);
                        chk2=armTouch.isPressed();
                    }
                }
                return true;
            }
        }

        public class Act1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0);
                intake.setPosition(1);
                telemetry.addLine("Act1 Intake action!");
                telemetry.update();
                return false;
            }
        }

        public class Act2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.5);
                telemetry.addLine("Act2 Intake action!");
                telemetry.update();
                return false;
            }
        }

        public class LiftUp2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double slpos = lift.getCurrentPosition();
                double armpos = arm.getCurrentPosition();
                if (chk3>3150 && chk3!=0){
                    telemetry.addLine("Ended liftUp2 action!");
                    telemetry.update();
                    chk2=false;
                    stage2=1;
                    return false;
                } else if (stage3==1){
                    telemetry.addLine("Stage1 liftUp2 action!");
                    telemetry.addLine("ArmPos: "+armpos);
                    telemetry.addLine("SlidePos: "+slpos);
                    telemetry.update();
                    arm.setTargetPosition(1000);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.7);
                    stage3 = 2;
                } else if (stage3==2){
                    if (armpos>970){
                        lift.setTargetPosition(3185);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                        telemetry.addLine("Stage2 liftUp2 action!");
                        telemetry.addLine("ArmPos: "+armpos);
                        telemetry.addLine("SlidePos: "+slpos);
                        telemetry.update();
                        stage3 = 3;
                    }
                } else if (stage3==3){
                    if (slpos>3170) {
                        wrist.setPosition(0);
                        tt = SystemClock.uptimeMillis();
                        while (SystemClock.uptimeMillis() - tt < 1000){
                            telemetry.addLine("Stage3 liftUp2 action!");
                        }
                        intake.setPosition(0);
                        tt = SystemClock.uptimeMillis();
                        while (SystemClock.uptimeMillis() - tt < 500){
                            telemetry.addLine("Stage4 liftUp2 action!");
                        }
                        wrist.setPosition(0.4);
                        tt = SystemClock.uptimeMillis();
                        while (SystemClock.uptimeMillis() - tt < 250){
                            telemetry.addLine("Stage6 liftUp2 action!");
                        }
                        telemetry.addLine("Stage4 liftUp2 action!");
                        telemetry.update();
                        chk3=lift.getCurrentPosition();
                    }
                }
                return true;
            }
        }

        public Action liftDown(){
            return new LiftDown();
        }

        public Action liftUp() {
            return new LiftUp();
        }
        public Action liftUp2() {
            return new LiftUp2();
        }

        public Action act1() {
            return new Act1();
        }

        public Action act2() {
            return new Act2();
        }

    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-11, -71, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(15, -46));
        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(-48, -55));
        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-48, -40));
        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-55, -70), Math.toRadians(223));
        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                //.waitSeconds(0.5)
                .strafeTo(new Vector2d(-50, -60));
        Action trajectoryActionCloseOut = tab5.endTrajectory().fresh()
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(-50, -5))
                .strafeTo(new Vector2d(-57, -5))
                .strafeTo(new Vector2d(-57, -62))
                .strafeTo(new Vector2d(-57, -5))
                .strafeTo(new Vector2d(-32, -3))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Action traj1;
        traj1 = tab1.build();
        Action traj2;
        traj2 = tab2.build();
        Action traj3;
        traj3 = tab3.build();
        Action traj4;
        traj4 = tab4.build();
        Action traj5;
        traj5 = tab5.build();

        Actions.runBlocking(
                new SequentialAction(
                        traj1,
                        lift.liftUp(),
                        lift.liftDown(),
                        traj2,
                        lift.act1(),
                        traj3,
                        lift.act2(),
                        traj4,
                        lift.liftUp2(),
                        traj5,
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}
