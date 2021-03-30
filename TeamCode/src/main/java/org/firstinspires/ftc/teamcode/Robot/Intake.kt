package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Util.HardwareNames


//Class that controls the front rollers of the robot
class Intake(robot: Robot): Component(robot) {



    //The motor that controls the rollers (no encoder)
    private val intakeMotor: DcMotorSimple
    private val intakeMotorTwo: DcMotorSimple



    init{
        intakeMotor = hardwareMap[DcMotorSimple::class.java, HardwareNames.Intake.INTAKE_MOTOR]
        intakeMotorTwo = hardwareMap[DcMotor::class.java, "x_encoder"]

        /*
        VERY IMPORTANT!!!!!!!!!!!!!! DO NOT CHANGE THE DIRECTION OF THIS MOTOR OTHERWISE IT WILL
        MESS WITH THE SHOOTER ENCODER!!!!!!!!

        If you need to change this use a negative sign on the motor power
         */

    }


    fun update(){
        //Emergency method of clearing a jam; hold a and left trigger to reverse rollers
        if(gamepad2.right_trigger > 0.7) startOutake()
        else if(gamepad2.left_trigger > 0.7) startIntake()
        else stop()
    }


    fun startIntake() = setIntakePower(-1.0)

    fun startOutake() = setIntakePower(1.0)

    fun stop() = setIntakePower(0.0)


    private fun setIntakePower(power: Double){
        intakeMotor.power = power
        intakeMotorTwo.power = power
    }

}