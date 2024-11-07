package org.firstinspires.ftc.teamcode.common.utils

import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import kotlin.math.floor

class Util {
    companion object {
        fun getAngleDiff(a: Pair<Vec2d, Double>, b: Vec2d): Double {
            val (aPoint, aAngle) = a
            val diff = Utils.normalizeAngle(aPoint.angleTo(b))
            return Utils.normalizeAngle(diff - aAngle)
        }


        // normalize radians to be between -pi and pi
        fun normalizeRadians(radians: Double): Double {
            val temp = (radians + Math.PI) / (2.0 * Math.PI)
            return (temp - floor(temp) - 0.5) * 2.0
        }

        fun normalizeDegrees(angle: Double): Double {
            return Math.toDegrees(normalizeRadians(Math.toRadians(angle)))
        }
    }
}