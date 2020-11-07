package org.firstinspires.ftc.teamcode.Util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class ServoTestingOpmode: LinearOpMode(){



    private lateinit var servos: ArrayList<Servo>

    @Throws(InterruptedException::class)
    override fun runOpMode(){

        servos = ArrayList()

        servos.add(hardwareMap.servo.get("servo 1"))
        servos.add(hardwareMap.servo.get("servo 2"))



        waitForStart()


        while (opModeIsActive()){

            if(gamepad1.a) servos.forEach {servo -> servo.position = 0.0}
            if(gamepad1.b) servos.forEach {servo -> servo.position = 1.0}

            if(gamepad1.x) servos.forEach {servo -> servo.position = gamepad1.right_stick_y.toDouble()}

        }
    }
}