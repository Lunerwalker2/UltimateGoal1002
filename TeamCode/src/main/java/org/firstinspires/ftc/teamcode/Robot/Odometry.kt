package org.firstinspires.ftc.teamcode.Robot

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.Odometry.OdometryLocalizer


class Odometry(robot: Robot, startingPose: Pose2d): Component(robot) {


    //The robot's pose in the field
    companion object {
        @JvmField
        var world_x: Double = 0.0

        @JvmField
        var world_y: Double = 0.0

        @JvmField
        var world_r: Double = 0.0
    }


    val localizer: OdometryLocalizer

    init {
        localizer = OdometryLocalizer(hardwareMap)

        localizer.poseEstimate = startingPose

        setPose(startingPose)
    }


    fun update(){
        localizer.update()

        world_x = localizer.poseEstimate.x
        world_y = localizer.poseEstimate.y
        world_r = localizer.poseEstimate.heading
    }


    fun setPose(newPose: Pose2d){
        localizer.poseEstimate = newPose

        world_x = newPose.x
        world_y = newPose.y
        world_r = newPose.heading
    }


}