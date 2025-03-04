package org.firstinspires.ftc.teamcode.common.utils

import android.os.Environment
import com.millburnx.utils.Path
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import java.io.File
import kotlin.math.floor

fun DcMotorEx.init(
    isForward: Boolean = true,
    isBrake: Boolean = false,
) {
    direction = if (isForward) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
    zeroPowerBehavior = if (isBrake) ZeroPowerBehavior.BRAKE else ZeroPowerBehavior.FLOAT
    reset()
}

fun DcMotorEx.reset() {
    mode = RunMode.STOP_AND_RESET_ENCODER
    mode = RunMode.RUN_WITHOUT_ENCODER
}

fun Servo.init(isForward: Boolean = true) {
    direction = if (isForward) Servo.Direction.FORWARD else Servo.Direction.REVERSE
}

fun ServoImplEx.init(
    isForward: Boolean = true,
    isAxon: Boolean = true,
) {
    direction = if (isForward) Servo.Direction.FORWARD else Servo.Direction.REVERSE
    if (isAxon) {
        pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }
}

fun CRServo.init(isForward: Boolean = true) {
    direction = if (isForward) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
}

fun CRServoImplEx.init(isForward: Boolean = true) {
    direction = if (isForward) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
    pwmRange = PwmControl.PwmRange(500.0, 2500.0)
}

fun normalizeRadians(radians: Double): Double {
    val temp = (radians + Math.PI) / (2.0 * Math.PI)
    return (temp - floor(temp) - 0.5) * 2.0
}

fun normalizeDegrees(angle: Double): Double = Math.toDegrees(normalizeRadians(Math.toRadians(angle)))

fun Path.Companion.loadPath(file: String): Path {
    val rootDir = Environment.getExternalStorageDirectory()
    val filePath = "$rootDir/Paths/$file.tsv"
    val path =
        try {
            val loaded = Vec2d.loadList(File(filePath))
            println(loaded)
            loaded
        } catch (e: Error) {
            e.printStackTrace()
            println("$file.tsv not found")
            Path(listOf())
        }
    return path
}
