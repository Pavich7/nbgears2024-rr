package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@TeleOp(name = "Servo Testing")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "wrist");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double rx = gamepad1.right_stick_x * 0.55;
            servo.setPosition(rx);
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}