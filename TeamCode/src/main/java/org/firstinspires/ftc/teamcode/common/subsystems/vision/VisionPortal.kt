package org.firstinspires.ftc.teamcode.common.subsystems.vision

import android.util.Size
import com.arcrobotics.ftclib.command.SubsystemBase
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor


class VisionPortal(hwMap: HardwareMap, camera: String, processors: List<VisionProcessor>) : SubsystemBase() {
    val cameraSize = Vec2d(640, 480)

    val builder = VisionPortal.Builder()
        .setCamera(hwMap.get(WebcamName::class.java, camera))
        .addProcessors(*processors.toTypedArray())
        .setCameraResolution(Size(cameraSize.x.toInt(), cameraSize.y.toInt()))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .enableLiveView(true)
        .setAutoStopLiveView(true)

    val visionPortal = builder.build()
}