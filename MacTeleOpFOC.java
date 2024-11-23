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

@TeleOp(name = "Full TeleOp Drive (FC)")
public class MacTeleOpFOC extends LinearOpMode {
    public Servo wrist;
    public Servo intake;
    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        DcMotor frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotor frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotor backRightMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotor RotateMotor = hardwareMap.get(DcMotorEx.class, "RotMot");
        DcMotor SlideMotor = hardwareMap.get(DcMotorEx.class, "SliMot");
        TouchSensor armTouch = hardwareMap.touchSensor.get("armTouch");
        TouchSensor slideTouch = hardwareMap.touchSensor.get("slideTouch");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(Servo.class, "intake");
        //Reset Encoder
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean InitSlideTouchPressed = !slideTouch.isPressed();
        while(!InitSlideTouchPressed){
            SlideMotor.setPower(0.3);
            InitSlideTouchPressed = !slideTouch.isPressed();
            telemetry.addLine("Init Slide: Status: "+InitSlideTouchPressed);
            telemetry.update();
        }
        boolean InitArmTouchPressed = armTouch.isPressed();
        while(!InitArmTouchPressed){
            RotateMotor.setPower(0.2);
            InitArmTouchPressed = armTouch.isPressed();
            telemetry.addLine("Init Arm: Status: "+InitArmTouchPressed);
            telemetry.update();
        }
        wrist.setPosition(0.35);
        SlideMotor.setPower(0);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RotateMotor.setPower(0);
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setTargetPosition(0);
        RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //End reset Encoder
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        boolean slimit = false;
        boolean isexitarm = false;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        opModeIsActive();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            int position = RotateMotor.getCurrentPosition();
            int positionsl = SlideMotor.getCurrentPosition();
            boolean ArmTouchPressed = armTouch.isPressed();
            boolean SlideTouchPressed = !slideTouch.isPressed();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 0.8;
            double rx = gamepad1.right_stick_x * 0.5;
            double roty = gamepad2.left_stick_y * 0.5;
            double ry2 = gamepad2.right_stick_y * 0.9;
            double lt = gamepad2.left_trigger;
            double rt = gamepad2.right_trigger;

            //Operator Rotate
            if (gamepad2.dpad_left) {
                rx=-0.25;
            } else if (gamepad2.dpad_right) {
                rx=0.25;
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            RotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Run using Encoder

            if(gamepad2.y && !isexitarm){
                RotateMotor.setTargetPosition(-750);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }
            if(gamepad2.x && !isexitarm){
                RotateMotor.setTargetPosition(0);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }
            if(gamepad2.b && !isexitarm){
                RotateMotor.setTargetPosition(-1500);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }
            if(gamepad2.a && !isexitarm){
                RotateMotor.setTargetPosition(-1800);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }
            if(gamepad2.left_bumper){
                intake.setPosition(0);
            }else if(gamepad2.right_bumper){
                intake.setPosition(1);
            }else{
                intake.setPosition(0.5);
            }
            if (gamepad1.a) {
                imu.resetYaw();
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            if(lt>0){
                wrist.setPosition(wrist.getPosition()-(lt*0.02));
            } else if (rt>0 && wrist.getPosition()<0.7) {
                wrist.setPosition(wrist.getPosition()+(rt*0.02));
            }

            //Check if in RUN_TO_POSITION
            if (roty > 0 || roty < 0){
                RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Limit Arm Position
                if(ArmTouchPressed && roty > 0) {
                    RotateMotor.setPower(0);
                }else if (position<-1800 && roty < 0) {
                    RotateMotor.setPower(0);
                }else{
                    RotateMotor.setPower(roty);
                }
                isexitarm=true;
            }else if (isexitarm && roty==0){
                RotateMotor.setTargetPosition(position);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
                isexitarm=false;
            }
            SlideMotor.setPower(ry2);

            // Limit Slide by Encoder

            if(SlideTouchPressed && ry2 > 0) {
                SlideMotor.setPower(0);
            }else if (positionsl<-2300 && ry2 < 0) {
                SlideMotor.setPower(0);
            }else{
                SlideMotor.setPower(ry2);
            }

            String rot = Double.toString(roty);
            String fL = Double.toString(frontLeftPower);
            String bL = Double.toString(backLeftPower);
            String fR = Double.toString(frontRightPower);
            String bR = Double.toString(backRightPower);
            String sl = Double.toString(ry2);
            telemetry.addLine("-------------Mecanum-------------");
            telemetry.addLine("frontLeft setPower: "+fL);
            telemetry.addLine("backLeftPower setPower: "+bL);
            telemetry.addLine("frontRightPower setPower: "+fR);
            telemetry.addLine("backRightPower setPower: "+bR);
            String bh = Double.toString(botHeading);
            telemetry.addLine("IMU BotHeading:"+bh);
            telemetry.addLine("-------------ArmDrive-------------");
            telemetry.addLine("SlideMotor setPower: "+sl);
            telemetry.addLine("RotateMotor setPower: "+rot);
            telemetry.addLine("-------------Gripper-------------");
            telemetry.addLine("Wrist setPosition: "+wrist.getPosition());
            telemetry.addLine("Intake setPosition: "+intake.getPosition());
            telemetry.addLine("-------------Encoder-------------");
            telemetry.addLine("Arm Encoder Position: "+position);
            telemetry.addLine("Slide Encoder Position: "+positionsl);
            telemetry.addLine("-------------Variable-------------");
            telemetry.addLine("IsExitArm: "+isexitarm);
            telemetry.addLine("ArmTouchPressed: "+ArmTouchPressed);
            telemetry.addLine("SlideTouchPressed: "+SlideTouchPressed);
            telemetry.update();
        }
    }
}
