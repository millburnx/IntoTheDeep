package com.millburnx.utils

import com.millburnx.utils.vec2d.Vec2d
import kotlin.math.pow

data class Bezier(val p0: Vec2d, val p1: Vec2d, val p2: Vec2d, val p3: Vec2d) {
    companion object {
        /**
         * @see <a href="https://stackoverflow.com/questions/30748316/catmull-rom-interpolation-on-svg-paths/30826434#30826434">Catmull Rom Derivation</a>
         */
        fun fromCatmullRom(p0: Vec2d, p1: Vec2d, p2: Vec2d, p3: Vec2d, alpha: Double = 0.5): Bezier {
            // equiv to t0..t3
            val knot0 = 0
            val knot1 = p0.distanceTo(p1).pow(alpha) + knot0
            val knot2 = p1.distanceTo(p2).pow(alpha) + knot1
            val knot3 = p2.distanceTo(p3).pow(alpha) + knot2

            // calculate derivative
            val c1 = (knot2 - knot1) / (knot2 - knot0)
            val c2 = (knot1 - knot0) / (knot2 - knot0)
            val d1 = (knot3 - knot2) / (knot3 - knot1)
            val d2 = (knot2 - knot1) / (knot3 - knot1)

            val m1 = ((p1 - p0) * c1 / (knot1 - knot0) + (p2 - p1) * c2 / (knot2 - knot1)) * (knot2 - knot1)
            val m2 = ((p2 - p1) * d1 / (knot2 - knot1) + (p3 - p2) * d2 / (knot3 - knot2)) * (knot2 - knot1)

            // control points
            val bezier0 = p1
            val bezier1 = p1 + m1 / 3
            val bezier2 = p2 - m2 / 3
            val bezier3 = p2

            return Bezier(bezier0, bezier1, bezier2, bezier3)
        }
    }

    fun at(t: Double): Vec2d {
        val segment1point1 = p0.lerp(p1, t)
        val segment1point2 = p1.lerp(p2, t)
        val segment1point3 = p2.lerp(p3, t)

        val segment2point1 = segment1point1.lerp(segment1point2, t)
        val segment2point2 = segment1point2.lerp(segment1point3, t)

        val segment3point = segment2point1.lerp(segment2point2, t)

        return segment3point
    }
}