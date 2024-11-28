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

@TeleOp(name = "Measurement (Encoder)")
public class EncoderMeasure extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor RotateMotor = hardwareMap.get(DcMotorEx.class, "RotMot");
        DcMotor SlideMotor = hardwareMap.get(DcMotorEx.class, "SliMot");
        TouchSensor armTouch = hardwareMap.touchSensor.get("armTouch");
        TouchSensor slideTouch = hardwareMap.touchSensor.get("slideTouch");
        //Reset Encoder
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean InitSlideTouchPressed = !slideTouch.isPressed();
        while(!InitSlideTouchPressed){
            SlideMotor.setPower(0.3);
            InitSlideTouchPressed = !slideTouch.isPressed();
            telemetry.addLine("Init Slide: Status: "+InitSlideTouchPressed);
            telemetry.addLine("Init Arm: Status: Pending...");
            telemetry.update();
        }
        boolean InitArmTouchPressed = armTouch.isPressed();
        while(!InitArmTouchPressed){
            RotateMotor.setPower(0.2);
            InitArmTouchPressed = armTouch.isPressed();
            telemetry.addLine("Init Slide: Status: OK");
            telemetry.addLine("Init Arm: Status: "+InitArmTouchPressed);
            telemetry.update();
        }

        SlideMotor.setPower(0);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setTargetPosition(0);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RotateMotor.setPower(0);
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setTargetPosition(0);
        RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //End reset Encoder
        telemetry.addLine("Init Slide: Status: OK");
        telemetry.addLine("Init Arm: Status: OK");
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();
        opModeIsActive();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            int position = RotateMotor.getCurrentPosition();
            int positionsl = SlideMotor.getCurrentPosition();
            boolean ArmTouchPressed = armTouch.isPressed();
            boolean SlideTouchPressed = !slideTouch.isPressed();

            if(gamepad2.a){
                RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addLine("====== Encoder Measurement ======");
            telemetry.addLine("Welcome! Use with caution!");
            telemetry.addLine("Press A to reset Encoder");
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