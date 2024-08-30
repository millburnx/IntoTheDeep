package com.millburnx.pathplanner

import com.millburnx.utils.Bezier
import com.millburnx.utils.BezierPoint

class PointManager {
    private val _points = mutableListOf<BezierPoint>()

    val beziers: List<Bezier> // like a computed ref
        get() {
            // turn points into catmull rom into beziers
            val beziers = mutableListOf<Bezier>()
            for (i in 0 until _points.size - 1) {
                val p1 = points[i]
                val p2 = points[i + 1]

                val p0 = points.getOrNull(i - 1) ?: BezierPoint(p1.anchor - (p2.anchor - p1.anchor))
                val p3 = points.getOrNull(i + 2) ?: BezierPoint(p2.anchor - (p1.anchor - p2.anchor))

                val bezier = Bezier.fromCatmullRom(p0.anchor, p1.anchor, p2.anchor, p3.anchor)
                _points[i].next = bezier.p1
                _points[i + 1].prev = bezier.p2
                beziers.add(bezier)
            }
            return beziers
        }

    val points: List<BezierPoint> // just a getter
        get() = _points

    fun addPoint(point: BezierPoint) {
        _points.add(point)
    }
}