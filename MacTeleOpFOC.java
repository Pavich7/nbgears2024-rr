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

@TeleOp(name = "Full TeleOp Drive")
public class MacTeleOpFOC extends LinearOpMode {
    public Servo wrist;
    public Servo intake;
    //public Servo cls1;
    //public Servo cls2;
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
        //cls1 = hardwareMap.get(Servo.class, "cls1");
        //cls2 = hardwareMap.get(Servo.class, "cls2");
        //Reset Encoder
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wrist.setPosition(0.35);
        //cls1.setPosition(0);
        //cls2.setPosition(0);
        //Slide Homing
        boolean InitSlideTouchPressed = !slideTouch.isPressed();
        while(!InitSlideTouchPressed){
            SlideMotor.setPower(0.3);
            InitSlideTouchPressed = !slideTouch.isPressed();
            if (gamepad1.x){
                break;
            }
            telemetry.addLine("Press X on Driver to EMERGENCY BYPASS homing.");
            telemetry.addLine("Init Slide: Status: "+InitSlideTouchPressed);
            telemetry.addLine("Init Arm: Status: Pending...");
            telemetry.update();
        }
        //Arm Homing
        boolean InitArmTouchPressed = armTouch.isPressed();
        while(!InitArmTouchPressed){
            RotateMotor.setPower(0.2);
            InitArmTouchPressed = armTouch.isPressed();
            if (gamepad1.x){
                break;
            }
            telemetry.addLine("Press X on Driver to EMERGENCY BYPASS homing.");
            telemetry.addLine("Init Slide: Status: OK");
            telemetry.addLine("Init Arm: Status: "+InitArmTouchPressed);
            telemetry.update();
        }

        //Reset Slide Encoder
        SlideMotor.setPower(0);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setTargetPosition(0);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Reset Arm Encoder
        RotateMotor.setPower(0);
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setTargetPosition(0);
        RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //End reset Encoder

        imu.resetYaw();
        telemetry.addLine("Initial homing processes completed!");
        telemetry.addLine("Init Slide: Status: OK");
        telemetry.addLine("Init Arm: Status: OK");
        telemetry.addLine("Robot is now ready.");
        telemetry.update();

        boolean isexitarm = false;
        boolean isexitsl = false;
        boolean outtaked = false;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        opModeIsActive();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Realtime Encoder and Limit Switch data
            int position = RotateMotor.getCurrentPosition();
            int positionsl = SlideMotor.getCurrentPosition();
            boolean ArmTouchPressed = armTouch.isPressed();
            boolean SlideTouchPressed = !slideTouch.isPressed();

            //Input value from joystick
            double y = -gamepad1.left_stick_y * 0.95; //Forward Backward
            double x = gamepad1.left_stick_x * 0.95; //Left Right
            double rx = gamepad1.right_stick_x * 0.5; //Rotate
            double roty = gamepad2.left_stick_y * 0.5; //Arm
            double ry2 = gamepad2.right_stick_y * 0.9; //Slide
            double lt = gamepad2.left_trigger;
            double rt = gamepad2.right_trigger;

            //Operator Rotate
            if (gamepad2.dpad_left) {
                rx=-0.25;
            } else if (gamepad2.dpad_right) {
                rx=0.25;
            }

            //FOC Driving
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            //Some Tuning?
            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            //Traditional Motor Break
            RotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Macros

            //Y High Basket
            if(gamepad2.y && !isexitarm && !isexitsl){
                if(RotateMotor.getCurrentPosition()<-1200 && RotateMotor.getCurrentPosition()>-1400){
                    SlideMotor.setTargetPosition(-3150);
                    SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideMotor.setPower(1);
                    wrist.setPosition(0.275);
                } else {
                    RotateMotor.setTargetPosition(-1230);
                    RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotateMotor.setPower(0.75);
                }
            }

            //X Home
            if(gamepad2.x && !isexitarm && !isexitsl){
                if(SlideMotor.getCurrentPosition()>-100){
                    RotateMotor.setTargetPosition(0);
                    RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotateMotor.setPower(0.75);
                } else {
                    SlideMotor.setTargetPosition(0);
                    SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideMotor.setPower(1);
                    wrist.setPosition(0.1);
                }
            }

            //A Pick up
            if(gamepad2.a && !isexitarm && !isexitsl){
                RotateMotor.setTargetPosition(-470);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.75);
                intake.setPosition(1);
                outtaked=false;
                wrist.setPosition(0);
            }

            //Intake-Outtake
            if(gamepad2.left_bumper){
                intake.setPosition(0);
                outtaked=true;
            }else if(lt>0){
                intake.setPosition(1);
                outtaked=false;
            }else{
                if (outtaked){
                    intake.setPosition(0.5);
                }
            }

            //Climber Support (Disabled)
            /*
            if(gamepad2.dpad_up){
                cls1.setPosition(0);
                cls2.setPosition(0);
            }else if(gamepad2.dpad_down){
                cls1.setPosition(1);
                cls2.setPosition(1);
            }
             */

            //Calibrate IMU
            if (gamepad1.a) {
                imu.resetYaw();
            }

            //Set Mecanum Wheels Speed
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            //Wrist Controlling
            if(rt>0 && wrist.getPosition()<0.465){
                wrist.setPosition(wrist.getPosition()+0.015);
            } else if (gamepad2.right_bumper) {
                wrist.setPosition(wrist.getPosition()-0.02);
            }

            //Hold Current, Arm Driving and Limits
            if (roty > 0 || roty < 0){
                RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Limit Arm Position
                if(ArmTouchPressed && roty > 0) {
                    RotateMotor.setPower(0);
                    RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }else if (position<-1800 && roty < 0) {
                    RotateMotor.setPower(0);
                }else{
                    RotateMotor.setPower(roty);
                }
                isexitarm=true;
            }else if (isexitarm && roty==0){
                //Switch to Hold Current mode
                RotateMotor.setTargetPosition(position);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
                isexitarm=false;
            }

            //Hold Current, Slide Driving and Limits
            if (ry2 > 0 || ry2 < 0){
                SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Limit Slide Position
                if(SlideTouchPressed && ry2 > 0) {
                    SlideMotor.setPower(0);
                    SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }else if (positionsl<-3150 && ry2 < 0) {
                    SlideMotor.setPower(0);
                }else if(positionsl<-2280 && ry2 < 0 && armTouch.isPressed()){
                    SlideMotor.setPower(0);
                }else{
                    SlideMotor.setPower(ry2);
                }
                isexitsl=true;
            }else if (isexitsl && ry2==0){
                //Switch to Hold Current mode
                SlideMotor.setTargetPosition(positionsl);
                SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideMotor.setPower(0.6);
                isexitsl=false;
            }

            //Telemetry Stuff

            //Telemetry value rounding
            String rot = String.format("%.7f",rotY);
            String fL = String.format("%.4f",frontLeftPower);
            String bL = String.format("%.4f",backLeftPower);
            String fR = String.format("%.4f",frontRightPower);
            String bR = String.format("%.4f",backRightPower);
            String sl = String.format("%.7f",ry2);
            String bh = String.format("%.7f",botHeading);

            //Telemetry Output
            telemetry.addLine("======== TeleOp Telemetry ========");
            telemetry.addLine("-------------Driving-------------");
            telemetry.addLine("frontLeft: "+fL+" frontRight: "+fR);
            telemetry.addLine("backLeft: "+bL+" backRight: "+bR);
            telemetry.addLine("Heading:"+bh);
            telemetry.addLine("-------------ArmDrive-------------");
            telemetry.addLine("SlideMotor setPower: "+sl);
            telemetry.addLine("RotateMotor setPower: "+rot);
            telemetry.addLine("-------------Gripper-------------");
            telemetry.addLine("Wrist setPosition: "+wrist.getPosition());
            telemetry.addLine("Intake setPosition: "+intake.getPosition());
            //telemetry.addLine("Climber setPosition: "+cls1.getPosition());
            telemetry.addLine("-------------Encoder-------------");
            telemetry.addLine("Arm Encoder Position: "+position);
            telemetry.addLine("Slide Encoder Position: "+positionsl);
            telemetry.addLine("-------------Variable-------------");
            //telemetry.addLine("IsExitArm: "+isexitarm);
            //telemetry.addLine("IsExitSlide: "+isexitsl);
            telemetry.addLine("ArmTouchPressed: "+ArmTouchPressed);
            telemetry.addLine("SlideTouchPressed: "+SlideTouchPressed);
            telemetry.update();
        }
    }
}
