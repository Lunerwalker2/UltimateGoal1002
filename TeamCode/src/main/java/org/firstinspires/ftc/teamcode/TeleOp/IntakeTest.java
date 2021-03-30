package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class IntakeTest extends LinearOpMode {






    @Override
    public void runOpMode() throws InterruptedException {


        DcMotorSimple intake = hardwareMap.dcMotor.get("left_y_encoder");
        DcMotorSimple intakeTwo = hardwareMap.dcMotor.get("x_encoder");



        waitForStart();

        while (opModeIsActive() && !isStopRequested()){

            if(gamepad1.left_bumper) {
                intake.setPower(-1.0);
                intakeTwo.setPower(-1.0);
            } else {
                intake.setPower(0);
                intakeTwo.setPower(0);
            }
        }
    }
}
