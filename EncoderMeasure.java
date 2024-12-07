package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Measurement & Self Test")
public class EncoderMeasure extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor RotateMotor = hardwareMap.get(DcMotorEx.class, "RotMot");
        DcMotor SlideMotor = hardwareMap.get(DcMotorEx.class, "SliMot");
        TouchSensor armTouch = hardwareMap.touchSensor.get("armTouch");
        TouchSensor slideTouch = hardwareMap.touchSensor.get("slideTouch");
        telemetry.addLine("Testing started.");
        telemetry.update();
        //Reset Encoder
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean InitSlideTouchPressed = !slideTouch.isPressed();
        while(!InitSlideTouchPressed){
            SlideMotor.setPower(0.3);
            InitSlideTouchPressed = !slideTouch.isPressed();
            telemetry.addLine("(1/5) Init Slide: Status: "+InitSlideTouchPressed);
            telemetry.update();
        }
        boolean InitArmTouchPressed = armTouch.isPressed();
        while(!InitArmTouchPressed){
            RotateMotor.setPower(0.2);
            InitArmTouchPressed = armTouch.isPressed();
            telemetry.addLine("(2/5) Init Arm: Status: "+InitArmTouchPressed);
            telemetry.update();
        }

        SlideMotor.setPower(0);
        RotateMotor.setPower(0);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SlideMotor.setTargetPosition(-300);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setPower(0.2);

        while (SlideMotor.getCurrentPosition() > -280){
            telemetry.addLine("(3/5) Testing Slide...");
            telemetry.update();
        }

        RotateMotor.setTargetPosition(-250);
        RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotateMotor.setPower(0.2);

        while (RotateMotor.getCurrentPosition() > -230){
            telemetry.addLine("(4/5) Testing Arm...");
            telemetry.update();
        }

        SlideMotor.setTargetPosition(0);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setPower(0.3);
        RotateMotor.setTargetPosition(0);
        RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotateMotor.setPower(0.2);

        while (!armTouch.isPressed() || slideTouch.isPressed()){
            telemetry.addLine("(5/5) Testing Limits...");
            telemetry.update();
        }

        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //End reset Encoder
        telemetry.addLine("Testing Completed!");
        telemetry.addLine("Ready for measurement.");
        telemetry.update();

        waitForStart();
        opModeIsActive();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            int position = RotateMotor.getCurrentPosition();
            int positionsl = SlideMotor.getCurrentPosition();
            boolean ArmTouchPressed = armTouch.isPressed();
            boolean SlideTouchPressed = !slideTouch.isPressed();

            if(gamepad1.a){
                RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addLine("====== Encoder Measurement ======");
            telemetry.addLine("Welcome! Use with caution!");
            telemetry.addLine("Press A on Gamepad 1 to reset Encoder");
            telemetry.addLine("-------------Encoder-------------");
            telemetry.addLine("Arm Encoder Position: "+position);
            telemetry.addLine("Slide Encoder Position: "+positionsl);
            telemetry.addLine("-------------Variable-------------");
            telemetry.addLine("ArmTouchPressed: "+ArmTouchPressed);
            telemetry.addLine("SlideTouchPressed: "+SlideTouchPressed);
            telemetry.update();
        }
    }
}
