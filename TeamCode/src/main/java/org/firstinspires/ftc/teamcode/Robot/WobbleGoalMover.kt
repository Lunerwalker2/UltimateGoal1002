package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util.HardwareNames

//Class that controls the wobble goal claw
class WobbleGoalMover(robot: Robot): Component(robot) {



    //Servo that controls arm
    private val wobbleGoalArm: Servo


    //Servo that controls claw
    private val wobbleGoalClaw: Servo


    companion object {

        //Servo position of the arm while it's up (and lifting the wobble goal above the border)
        const val armUpPosition = 0.4

        // Servo position of the arm when it's slightly above horizontal
        const val armHorizontalPosition = 0.0

        const val clawOpenPosition = 0.7

        const val clawClosePosition = 0.3



        //Servo position of the arm when it's just below
    }


    init {
        wobbleGoalArm = hardwareMap[Servo::class.java, HardwareNames.WobbleGoal.WOBBLE_ARM]

        wobbleGoalClaw = hardwareMap[Servo::class.java, HardwareNames.WobbleGoal.WOBBLE_CLAW]

    }



    var armUp = true

    var clawOpen = false

    fun update() {

        if(armUp && wobbleGoalArm.position != armUpPosition){
            wobbleGoalArm.position = armUpPosition
        } else if(wobbleGoalArm.position != armHorizontalPosition){
            wobbleGoalArm.position = armHorizontalPosition
        }



        if(clawOpen && wobbleGoalClaw.position != clawOpenPosition){
            wobbleGoalClaw.position = clawOpenPosition
        } else if(wobbleGoalClaw.position != clawClosePosition){
            wobbleGoalClaw.position = clawClosePosition
        }
    }



}