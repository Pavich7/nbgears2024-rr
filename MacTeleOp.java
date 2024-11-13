package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Full TeleOp Drive (RC)")
public class MacTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotor frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotor backRightMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotor RotateMotor = hardwareMap.get(DcMotorEx.class, "RotMot");
        DcMotor SlideMotor = hardwareMap.get(DcMotorEx.class, "SliMot");
        TouchSensor armTouch = hardwareMap.touchSensor.get("armTouch");
        TouchSensor slideTouch = hardwareMap.touchSensor.get("slideTouch");
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
            double ry2= gamepad2.right_stick_y * 0.8;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.8);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            RotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Run using Encoder

            if(gamepad2.y && !isexitarm){
                RotateMotor.setTargetPosition(-1000);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }
            if(gamepad2.x && !isexitarm){
                RotateMotor.setTargetPosition(-100);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }
            if(gamepad2.b && !isexitarm){
                RotateMotor.setTargetPosition(-1500);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }
            if(gamepad2.a && !isexitarm){
                RotateMotor.setTargetPosition(-2200);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            //Check if in RUN_TO_POSITION
            if (roty > 0 || roty < 0){
                RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Limit Arm Position
                if(ArmTouchPressed && roty > 0) {
                    RotateMotor.setPower(0);
                }else if (position<-2340 && roty < 0) {
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
            telemetry.addLine("-------------ArmDrive-------------");
            telemetry.addLine("SlideMotor setPower: "+sl);
            telemetry.addLine("RotateMotor setPower: "+rot);
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
