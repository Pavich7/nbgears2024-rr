package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Mecanum & Rotate Drive Testing")
public class MacTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotor frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotor backRightMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotor RotateMotor = hardwareMap.get(DcMotorEx.class, "RotMot");
        TouchSensor armTouch = hardwareMap.touchSensor.get("armTouch");
        //Reset Encoder
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean InitArmTouchPressed = armTouch.isPressed();
        while(InitArmTouchPressed==false){
            RotateMotor.setPower(0.3);
            InitArmTouchPressed = armTouch.isPressed();
            telemetry.addLine("Init Arm: Status: "+InitArmTouchPressed);
            telemetry.update();
        }
        RotateMotor.setPower(0);
        RotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateMotor.setTargetPosition(0);
        RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //End reset Encoder
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        opModeIsActive();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            int position = RotateMotor.getCurrentPosition();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 0.8;
            double rx = gamepad1.right_stick_x * 0.5;
            //double roty = gamepad2.left_stick_y * 0.5;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.8);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            RotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Run using Encoder
            if(gamepad2.y){
                RotateMotor.setTargetPosition(-1000);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }
            if(gamepad2.x){
                RotateMotor.setTargetPosition(-100);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }
            if(gamepad2.b){
                RotateMotor.setTargetPosition(-1500);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }
            if(gamepad2.a){
                RotateMotor.setTargetPosition(-2200);
                RotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RotateMotor.setPower(0.6);
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            //RotateMotor.setPower(roty);

            //String rot = Double.toString(roty);
            String fL = Double.toString(frontLeftPower);
            String bL = Double.toString(backLeftPower);
            String fR = Double.toString(frontRightPower);
            String bR = Double.toString(backRightPower);
            //telemetry.addLine("Rotate setPower: "+rot);
            telemetry.addLine("frontLeft setPower: "+fL);
            telemetry.addLine("backLeftPower setPower: "+bL);
            telemetry.addLine("frontRightPower setPower: "+fR);
            telemetry.addLine("backRightPower setPower: "+bR);
            telemetry.addLine("--------------------------");
            telemetry.addLine("Encoder Position: "+position);
            telemetry.update();
        }
    }
}
