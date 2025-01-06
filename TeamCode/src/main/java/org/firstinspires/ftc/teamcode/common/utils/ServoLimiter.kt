package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp

open class ServoLimiter(var maxSpeed: Double, val getDelta: () -> Double, startingPosition: Double = 0.0) {
    var current: Double = startingPosition
    var target: Double = startingPosition

    fun update(target: Double = this.target) {
        val speedLimit = maxSpeed * getDelta()
        this.current += (target - this.target).clamp(-speedLimit, speedLimit)
    }
}