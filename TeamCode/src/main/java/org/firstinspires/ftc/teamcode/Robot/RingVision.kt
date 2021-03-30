package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Vision.pipelines.RingStackPipeline

import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam


class RingVision(robot: Robot): Component(robot) {




    val camera: OpenCvWebcam

    val pipeline = RingStackPipeline()

    companion object {
        var currentDeliveryZone = RingStackPipeline.DeliveryZone.A
    }


    init {

        val cameraMonitorViewId = opMode.hardwareMap.appContext.resources
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        opMode.hardwareMap.appContext.packageName
                )

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(opMode.hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)


    }

    fun update(){

        telemetry.addData("Frame Count", camera.frameCount)
        telemetry.addData("FPS", String.format("%.2f", camera.fps))
        telemetry.addData("Total frame time ms", camera.totalFrameTimeMs)
        telemetry.addData("Pipeline time ms", camera.pipelineTimeMs)
        telemetry.addData("Overhead time ms", camera.overheadTimeMs)
        telemetry.addData("Theoretical max FPS", camera.currentPipelineMaxFps)
        telemetry.addData("Zone", RingStackPipeline.deliveryZone)
        telemetry.update();


        currentDeliveryZone = RingStackPipeline.deliveryZone
    }

    fun startStream() {
        camera.setPipeline(pipeline)

        camera.openCameraDeviceAsync {
            camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        }

        dashboard.startCameraStream(camera, 30.0)
    }

    fun stopStream() {
        dashboard.stopCameraStream()
        camera.stopStreaming()
        camera.closeCameraDevice()
    }




}