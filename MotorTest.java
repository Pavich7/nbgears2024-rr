package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Motor Testing")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor SlideMotor = hardwareMap.get(DcMotorEx.class, "SliMot");

        waitForStart();
        opModeIsActive();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double ry2 = gamepad2.right_stick_y * 0.8;
            SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SlideMotor.setPower(ry2);
            String sl = Double.toString(ry2);
            telemetry.addLine("SlideMotor setPower: "+sl);
            telemetry.update();
        }
    }
}