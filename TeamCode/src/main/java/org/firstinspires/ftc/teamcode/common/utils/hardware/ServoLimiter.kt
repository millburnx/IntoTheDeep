package org.firstinspires.ftc.teamcode.common.utils.hardware

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import kotlin.math.abs
import kotlin.math.roundToLong

open class ServoLimiter(
    var maxSpeed: Double,
    val getDelta: () -> Double,
    startingPosition: Double = 0.0,
) {
    var current: Double = startingPosition

    fun update(target: Double) {
        if (maxSpeed == -1.0) {
            current = target
            return
        }
        val speedLimit = maxSpeed * getDelta()
        val newCurrent = current + (target - this.current).clamp(-speedLimit, speedLimit)
        current = newCurrent.clamp(0, 1)
    }

    companion object {
        fun estimateDuration(
            starting: Double,
            ending: Double,
            speed: Double,
        ): Long {
            val diff = abs(starting - ending)
            return (diff / speed * 1000).roundToLong()
        }
    }
}
