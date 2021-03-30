package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import kotlin.jvm.Throws


@TeleOp
class HopperTesting: LinearOpMode() {




    lateinit var hopper: Servo

    lateinit var flicker: Servo


    @Throws(InterruptedException::class)
    override fun runOpMode() {

        hopper = hardwareMap.servo["hopper_servo"]

        flicker = hardwareMap.servo["flicker"]



        waitForStart()

        while(opModeIsActive()){
            if (isStopRequested) return;


            if(gamepad1.a) hopper.position = 0.03
            else if(gamepad1.b) hopper.position = 0.2

            if(gamepad1.x) flicker.position = 0.6
            else if(gamepad1.y) flicker.position = 0.0
        }
    }
}