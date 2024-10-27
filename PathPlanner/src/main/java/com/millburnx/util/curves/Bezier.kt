package com.millburnx.util.curves

import com.millburnx.util.Circle
import com.millburnx.util.IVec2d
import com.millburnx.util.Math
import com.millburnx.util.Vec2d
import com.millburnx.util.Vector2d
import kotlin.math.abs

public abstract class Bezier : Curve {
    public abstract override val points: List<Vec2d>

    public override fun at(t: Double): Vec2d {
        // De Casteljau's algorithm
        var input = points
        var output: Array<Vec2d>? = null

        while (input.size > 1) {
            output = Array(input.size - 1) { Vector2d() }
            input.zipWithNext().forEachIndexed { index, (a, b) ->
                output[index] = Vector2d(
                    Math.lerp(a.x, b.x, t),
                    Math.lerp(a.y, b.y, t)
                )
            }
            input = output.toList()
        }

        return input[0]
    }
}

public data class LinearBezier(public val start: Vec2d, public val end: Vec2d) : Bezier() {
    public constructor(start: IVec2d, end: IVec2d) : this(Vec2d(start), Vec2d(end))

    override val points: List<Vec2d> = listOf(start, end)
}

public data class QuadraticBezier(public val start: Vec2d, public val control: Vec2d, public val end: Vec2d) :
    Bezier() {
    public constructor(start: IVec2d, control: IVec2d, end: IVec2d) : this(Vec2d(start), Vec2d(control), Vec2d(end))

    override val points: List<Vec2d> = listOf(start, control, end)
}

public data class CubicBezier(
    public val start: Vec2d,
    public val control1: Vec2d,
    public val control2: Vec2d,
    public val end: Vec2d
) : Bezier() {
    public constructor(start: IVec2d, control1: IVec2d, control2: IVec2d, end: IVec2d) : this(
        Vector2d(start),
        Vector2d(control1),
        Vector2d(control2),
        Vector2d(end)
    )

    public fun getLUT(samples: Int = 100): List<Vec2d> {
        val points = mutableListOf<Vec2d>()
        for (i in 0..samples) {
            val t = i.toDouble() / samples
            val point = at(t)
            points.add(point)
        }
        return points
    }

    public fun intersections(circle: Circle): List<Vec2d> {
        val LUT = getLUT().map { it -> it to null as Double? }.toMutableList()
        var start = 0
        val values = mutableListOf<Int>()

        for (count in 0..25) {
            val i = start + findClosest(
                circle.center,
                LUT,
                start,
                LUT.getOrNull(start - 2)?.second,
                LUT.getOrNull(start - 1)?.second,
                circle.radius
            )
            if (i < start) break
            if (i > 0 && i == start) break
            values.add(i)
            start = i + 2
        }
        return values.map { LUT[it].first }
    }

    public fun findClosest(
        target: Vec2d,
        LUT: MutableList<Pair<Vec2d, Double?>>,
        start: Int,
        pd2: Double?,
        pd1: Double?,
        radius: Double,
        distanceEpsilon: Int = 5
    ): Int {
        var distance = Double.MAX_VALUE
        var prevDistance2 = pd2 ?: distance
        var prevDistance1 = pd1 ?: distance
        var i = -1

        val sliced = LUT.slice(start until LUT.size)
        for (i in sliced.indices) {
            val p = sliced[i]
            val dist = abs(p.first.distanceTo(Vec2d(target.x, target.y)) - radius)
//            sliced[i] = p.first to dist
            LUT[i + start] = p.first to dist // offset th slice

            if (prevDistance1 < distanceEpsilon && prevDistance2 > prevDistance1 && prevDistance1 < dist) {
                return i - 1
            }

            if (dist < distance) {
                distance = dist
            }

            prevDistance2 = prevDistance1
            prevDistance1 = dist
        }

        return i;
    }

    override val points: List<Vec2d> = listOf(start, control1, control2, end)
}
