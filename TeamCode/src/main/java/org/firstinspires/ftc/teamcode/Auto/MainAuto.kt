package org.firstinspires.ftc.teamcode.Auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Robot.DriveFields
import org.firstinspires.ftc.teamcode.Robot.RingVision
import org.firstinspires.ftc.teamcode.Robot.Robot
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Vision.pipelines.RingStackPipeline
import java.lang.Math.toRadians
import kotlin.jvm.Throws


@Autonomous
class MainAuto(): AutoBase() {


    private val timer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

    @Throws(InterruptedException::class)
    override fun runOpMode(){
        robot = Robot(this, Alliance.REMOTE_AUTO)

        waitForStart()


        sleep(400)

        timer.reset()

        DriveFields.movement_y = 1.0
        while(timer.milliseconds() < 2000){
            robot.update()
        }
        robot.end()







    }

}