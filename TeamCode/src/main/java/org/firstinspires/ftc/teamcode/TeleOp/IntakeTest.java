package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class IntakeTest extends LinearOpMode {








    @Override
    public void runOpMode() throws InterruptedException {


        DcMotorSimple intake = hardwareMap.dcMotor.get("left_y_encoder");



        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            intake.setPower(-gamepad1.left_stick_y);
        }
    }
}
