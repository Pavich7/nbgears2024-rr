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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
@Autonomous(name = "Auto Drive Test Blue", group = "Autonomous")
public class AutoDriveTest extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift;
        private DcMotorEx arm;
        TouchSensor slideTouch = hardwareMap.touchSensor.get("slideTouch");
        TouchSensor armTouch = hardwareMap.touchSensor.get("armTouch");
        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "SliMot");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            arm = hardwareMap.get(DcMotorEx.class, "RotMot");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LiftUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double slpos = lift.getCurrentPosition();
                double armpos = arm.getCurrentPosition();

                if (slpos >= 1560 && slpos <= 1570 && armpos >= 815 && armpos <= 825){
                    telemetry.addLine("Ended liftUp action!");
                    telemetry.update();
                    return false;
                } else {
                    telemetry.addLine("Stage1 liftUp action!");
                    telemetry.addLine("ArmPos: "+armpos);
                    telemetry.addLine("SlidePos: "+slpos);
                    telemetry.update();
                    arm.setTargetPosition(819);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.6);
                    if (armpos>800){
                        lift.setTargetPosition(1567);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0.6);
                        telemetry.addLine("Stage2 liftUp action!");
                        telemetry.addLine("ArmPos: "+armpos);
                        telemetry.addLine("SlidePos: "+slpos);
                        telemetry.update();
                    }
                    return true;
                }
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

                if (!slideTouch.isPressed() && armTouch.isPressed()){
                    telemetry.addLine("Ended liftDown action!");
                    telemetry.update();
                    return false;
                } else {;
                    telemetry.addLine("Stage1 liftDown action!");
                    telemetry.addLine("ArmPos: "+armpos);
                    telemetry.addLine("SlidePos: "+slpos);
                    telemetry.update();
                    lift.setTargetPosition(1);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(0.6);
                    if (!slideTouch.isPressed()){
                        arm.setTargetPosition(1);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.6);
                        telemetry.addLine("Stage2 liftDown action!");
                        telemetry.addLine("ArmPos: "+armpos);
                        telemetry.addLine("SlidePos: "+slpos);
                        telemetry.update();
                    }
                    return true;
                }
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

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(8, -46))
                .waitSeconds(1);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .waitSeconds(1)
                .strafeTo(new Vector2d(8, -56))
                //.strafeTo(new Vector2d(38.89, -31.59))
                //.strafeTo(new Vector2d(37.29, -11.84))
                //.strafeTo(new Vector2d(47.79, -11.48))
                //.turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(48.33, -67.55))
                //.strafeTo(new Vector2d(47.79, -8.81))
                //.strafeTo(new Vector2d(57.23, -8.10))
                //.strafeTo(new Vector2d(58.29, -65.41))
                //.strafeTo(new Vector2d(57.05, -5.61))
                //.strafeTo(new Vector2d(65.95, -5.78))
                //.strafeTo(new Vector2d(68.44, -65.06))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

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
