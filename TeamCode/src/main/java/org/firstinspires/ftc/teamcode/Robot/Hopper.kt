package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Util.HardwareNames

class Hopper(robot: Robot): Component(robot) {




    private val hopperServo: Servo

    private val flicker: Servo



    companion object {

        const val hopperDownPosition = 0.2
        const val hopperUpPosition = 0.02

        const val flickerOutPosition = 0.6

        const val flickerInPosition = 0.0


        //How long it takes to move from in to out
        const val flickingTime = 600.0

    }


    init {

        hopperServo = hardwareMap[Servo::class.java, HardwareNames.Hopper.HOPPER_SERVO]

        flicker = hardwareMap[Servo::class.java, HardwareNames.Hopper.FLICKER]
    }


    var hopperUp = false


    private val flickerTimer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)


    // Basically, check each time if we are flicking and if so, what position are we going to and
    // how long has it been since it started

    fun update(){

        if(hopperUp && hopperServo.position != hopperUpPosition){
            hopperServo.position = hopperUpPosition
        } else if(!hopperUp && hopperServo.position != hopperDownPosition){
            hopperServo.position = hopperDownPosition
        }

    }


    fun flick(out: Boolean) {
        if(out) flicker.position = flickerOutPosition
        else flicker.position = flickerInPosition
    }



}