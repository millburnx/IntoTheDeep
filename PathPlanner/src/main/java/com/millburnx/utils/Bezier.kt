package com.millburnx.utils

import com.acmerobotics.dashboard.canvas.Canvas
import java.awt.Color
import java.awt.Graphics2D
import kotlin.math.pow

data class boundingData(val min: Vec2d, val max: Vec2d, val xRoots: List<Double>, val yRoots: List<Double>)

data class Bezier(val p0: Vec2d, val p1: Vec2d, val p2: Vec2d, val p3: Vec2d) {
    companion object {
        fun fromLine(p0: Vec2d, p1: Vec2d): Bezier {
            return Bezier(p0, p0.lerp(p1, 1.0 / 3), p1.lerp(p0, 1.0 / 3), p1)
        }

        /**
         * Generates a cubic bezier from a catmull-rom spline,
         * @param p0 the point before the start of the curve
         * @param p1 the start of the curve
         * @param p2 the end of the curve
         * @param p3 the point after the end of the curve
         * @param alpha knot parameter, 0.0 for a normal uniform cubic bezier,
         * @see <a href="https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline">Centripetal Catmull-Rom spline - Wikipedia</a>
         */
        fun fromCatmullRom(p0: Vec2d, p1: Vec2d, p2: Vec2d, p3: Vec2d, alpha: Double = 0.0): Bezier {
            val t0 = 0
            val t1 = p0.distanceTo(p1).pow(alpha) + t0
            val t2 = p1.distanceTo(p2).pow(alpha) + t1
            val t3 = p2.distanceTo(p3).pow(alpha) + t2

            val c1 = (t2 - t1) / (t2 - t0)
            val c2 = (t1 - t0) / (t2 - t0)
            val d1 = (t3 - t2) / (t3 - t1)
            val d2 = (t2 - t1) / (t3 - t1)

            val v1 = ((p1 - p0) * c1 / (t1 - t0) + (p2 - p1) * c2 / (t2 - t1)) * (t2 - t1) // velocity/derivative at p1
            val v2 = ((p2 - p1) * d1 / (t2 - t1) + (p3 - p2) * d2 / (t3 - t2)) * (t2 - t1) // velocity/derivative at p2

            return Bezier(p1, p1 + v1 / 3, p2 - v2 / 3, p2)
        }
    }

    /**
     * Get the point on the Bézier curve at t (0.0 to 1.0) using De Casteljau's algorithm
     * @see <a href="https://youtu.be/jvPPXbo87ds?t=234">De Casteljau's algorithm - Freya Holmér "The Continuity of Splines" @3:54 - YouTube</a>
     */
    fun at(t: Double): Vec2d {
        val s0p0 = p0.lerp(p1, t)
        val s0p1 = p1.lerp(p2, t)
        val s0p2 = p2.lerp(p3, t)

        val s1p0 = s0p0.lerp(s0p1, t)
        val s1p1 = s0p1.lerp(s0p2, t)

        return s1p0.lerp(s1p1, t)
    }

    /**
     * Get a list of intersections between the Bézier curve and a circle
     * @param samples the number of segments to split the Bézier curve into; curve resolution; higher is more accurate but slower
     */
    fun intersections(
        circle: Circle,
        canvas: Canvas? = null,
        iterations: Int = 4,
        parent: Bezier = this,
        start: Double = 0.0,
        end: Double = 1.0
    ): List<BezierIntersection> {
        if (iterations == 0) {
            // slow line intersection check
            val intersections = mutableListOf<BezierIntersection>()
            var lastPoint = 0.0;
            for (i in 1..10) {
                val t = i.toDouble() / 10
                val segment = LineSegment(at(lastPoint), at(t))
                val intersection = segment.intersections(circle)
                val realT = Utils.lerp(start, end, t)
                intersections.addAll(intersection.map { BezierIntersection(it.point, parent, realT) })
                lastPoint = t
            }
            return intersections
        }
        // bounding check
        val bounding = getBoundingTight()
        canvas?.setStroke("#FF0000")?.strokeRect(
            bounding.min.x,
            bounding.min.y,
            bounding.max.x - bounding.min.x,
            bounding.max.y - bounding.min.y
        )
        val circleBounding = circle.bounding()
        val intersects = Utils.boundingIntersection(bounding.min to bounding.max, circleBounding)
        if (!intersects) return listOf()
        // split the curve into 2
        val center = Utils.lerp(start, end, 0.5)
        val (left, right) = this.split()
        // recursive binary almost
        return left.intersections(circle, canvas, iterations - 1, parent, start, center) +
                right.intersections(circle, canvas, iterations - 1, parent, center, end)
    }

    fun split(t: Double = 0.5): Pair<Bezier, Bezier> {
        // de Casteljau's blossoming
        val p01 = p0.lerp(p1, t)
        val p12 = p1.lerp(p2, t)
        val p23 = p2.lerp(p3, t)
        val p012 = p01.lerp(p12, t)
        val p123 = p12.lerp(p23, t)
        val p0123 = p012.lerp(p123, t)
        return Bezier(p0, p01, p012, p0123) to Bezier(p0123, p123, p23, p3)
    }

    fun getBoundingTight(): boundingData {
        val aTemp = p0 * -3 + p1 * 9 - p2 * 9 + p3 * 3
        // avoid division by zero
        val a =
            Vec2d(if (aTemp.x == 0.0) 0.0001 else aTemp.x, if (aTemp.y == 0.0) 0.0001 else aTemp.y)
        val b = p0 * 6 - p1 * 12 + p2 * 6
        val c = p0 * -3 + p1 * 3
        val tx = Utils.quadraticFormula(a.x, b.x, c.x)
        val ty = Utils.quadraticFormula(a.y, b.y, c.y)
        val xRoots =
            listOf(tx?.first, tx?.second).filterNotNull()
                .filter { it in 0.0..1.0 }
        val yRoots =
            listOf(ty?.first, ty?.second).filterNotNull()
                .filter { it in 0.0..1.0 }

        val possible = listOf(p0, p3) + xRoots.map { at(it) } + yRoots.map { at(it) }
        val minX = possible.minOf { it.x }
        val minY = possible.minOf { it.y }
        val maxX = possible.maxOf { it.x }
        val maxY = possible.maxOf { it.y }

        return boundingData(
            Vec2d(minX, minY),
            Vec2d(maxX, maxY),
            xRoots,
            yRoots
        )
    }

    /**
     * Draw the Bézier curve on an FTC Dashboard canvas
     */
    fun draw(canvas: Canvas, samples: Int = 100) {
        var lastPoint = p0
        for (i in 1..samples) {
            val t = i.toDouble() / samples
            val point = at(t)
            canvas.strokeLine(lastPoint.x, lastPoint.y, point.x, point.y)
            lastPoint = point
        }
    }

    fun g2dDraw(g2d: Graphics2D, ppi: Double, scale: Double, color: Color) {
        g2d.color = color
        val samples = 100
        for (i in 1..samples) {
            val t = i.toDouble() / samples
            val point = at(t)
            val lastPoint = at((i - 1).toDouble() / samples)
            g2d.drawLine(
                (lastPoint.x * ppi).toInt(),
                (lastPoint.y * ppi).toInt(),
                (point.x * ppi).toInt(),
                (point.y * ppi).toInt()
            )
        }
    }
}