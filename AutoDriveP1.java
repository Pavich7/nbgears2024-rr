package org.firstinspires.ftc.teamcode;
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
@Autonomous(name = "Full Auto Drive (P1)")
public class AutoDriveP1 extends LinearOpMode {
    public Servo wrist;
    public Servo intake;
    public class Lift {
        private DcMotorEx lift;
        private DcMotorEx arm;
        double chk1 = 0;
        double stage1 = 1;
        boolean chk2 = false;
        double stage2 = 1;
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
                    arm.setPower(0.6);
                    wrist.setPosition(0.465);
                    intake.setPosition(0.5);
                    stage1 = 2;
                } else if (stage1==2){
                    if (armpos>800){
                        lift.setTargetPosition(1445);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0.6);
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

        public Action liftUp() {
            return new LiftUp();
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
                    lift.setPower(0.6);
                    stage2=2;
                } else if (stage2==2){
                    if (!slideTouch.isPressed() || lift.getCurrentPosition()<=2){
                        arm.setTargetPosition(1);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.6);
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

        public Action liftDown(){
            return new LiftDown();
        }

    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(8, -71, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(8, -46));
                //.waitSeconds(0.5);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                //.waitSeconds(0.5)
                //Go to push point and turn
                .strafeTo(new Vector2d(8, -46))
                .strafeTo(new Vector2d(55, -46))
                .strafeTo(new Vector2d(55, -26))
                .turn(Math.toRadians(-90))
                //Sample Push 1
                .strafeTo(new Vector2d(55, -26))
                .strafeTo(new Vector2d(55, -90))
                .strafeTo(new Vector2d(55, -26))
                //Sample Push 2
                .strafeTo(new Vector2d(65, -26))
                .strafeTo(new Vector2d(65, -90))
                .strafeTo(new Vector2d(65, -26))
                //Sample Push 3
                .strafeTo(new Vector2d(72.5, -26))
                .strafeTo(new Vector2d(72.5, -95))
                .turn(Math.toRadians(90))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}
