package com.millburnx.util.geometry.curves

import com.millburnx.util.Math
import com.millburnx.util.geometry.Circle
import com.millburnx.util.geometry.IVec2d
import com.millburnx.util.geometry.Vec2d
import com.millburnx.util.geometry.Vector2d
import kotlin.math.abs

public abstract class Bezier : Curve {
    public abstract override val points: List<Vec2d>

    public override fun at(t: Double): Vec2d {
        // De Casteljau's algorithm
        var input = points
        var output: MutableList<Vec2d> = mutableListOf()

        while (input.size > 1) {
            output = mutableListOf<Vec2d>()
            input.zipWithNext().forEach { (a, b) ->
                output.add(
                    Vector2d(
                        Math.lerp(a.x, b.x, t),
                        Math.lerp(a.y, b.y, t)
                    )
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

    public fun getLUT(samples: Int = 100): Pair<List<Vec2d>, Double> {
        val points = mutableListOf<Vec2d>()
        var distanceEpsilon = Double.MAX_VALUE
        for (i in 0..samples) {
            val t = i.toDouble() / samples
            val point = at(t)
            points.add(point)
            if (i > 0) {
                val last = points[i - 1]
                val dist = abs(point.distanceTo(last))
                if (dist < distanceEpsilon) {
                    distanceEpsilon = dist
                }
            }
        }
        return Pair(points, distanceEpsilon)
    }

    val lut: Pair<List<Vec2d>, Double> by lazy { getLUT() }

    public fun intersections(circle: Circle, samples: Int = 25): List<Vec2d> {
        val distances: Array<Double> = Array(lut.first.size) { Double.MAX_VALUE }
        var start = 0
        val values = mutableListOf<Int>()
        val distanceEpsilon = lut.second

        val findClosest = fun(): Int {
            // the largest point-to-point distance in our LUT
            var minDist = Double.MAX_VALUE
            var prevDist1 = distances.getOrNull(start - 1) ?: Double.MAX_VALUE
            var prevDist2 = distances.getOrNull(start - 2) ?: Double.MAX_VALUE

//            val sliced = lut.first.slice(start until lut.first.size)
            for (i in start until lut.first.size) {
//                val p = sliced[i]
                val p = lut.first[i]
//                distances[i + start] = dist
//                distances[i] = dist
                val dist = if (distances[i] == Double.MAX_VALUE) {
                    val dist = abs(p.distanceTo(Vec2d(circle.center.x, circle.center.y)) - circle.radius)
                    distances[i] = dist
                    dist
                } else {
                    distances[i]
                }

                if (prevDist1 < distanceEpsilon && prevDist2 > prevDist1 && prevDist1 < dist) {
                    return i - 1
                }

                if (dist < minDist) {
                    minDist = dist
                }

//                prevDist = dist to prevDist.first
                prevDist2 = prevDist1
                prevDist1 = dist
            }

            return -1
        }

        repeat(samples) {
            val i = findClosest()
            if (i < start || (i > 0 && i == start)) return values.map { lut.first[it] }
            values.add(i)
            start = i + 2
        }
        return values.map { lut.first[it] }
    }

    override val points: List<Vec2d> by lazy { listOf(start, control1, control2, end) }
}