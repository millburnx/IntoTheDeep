package com.millburnx.util.geometry.curves

import com.millburnx.util.geometry.Vector2d
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals

class HermiteTest {
    @Test
    fun `Correct Values`() {
        val p0 = Vector2d(0.0, 0.0)
        val v0 = Vector2d(1.0, 0.0)
        val p1 = Vector2d(1.0, 1.0)
        val v1 = Vector2d(1.0, 0.0)
        val hermite = Hermite(p0, v0, p1, v1)

        assert(hermite.at(0.0) == p0)
        assert(hermite.at(1.0) == p1)

        assert(hermite.at(0.5) == Vector2d(0.5, 0.5))

        assert(hermite.at(0.25) == Vector2d(0.25, 0.15625))
        assert(hermite.at(0.75) == Vector2d(0.75, 0.84375))
    }

    @Test
    fun `Correct Bezier`() {
        val p0 = Vector2d(0.0, 0.0)
        val v0 = Vector2d(1.0, 0.0)
        val p1 = Vector2d(1.0, 1.0)
        val v1 = Vector2d(1.0, 0.0)
        val hermite = Hermite(p0, v0, p1, v1)
        val bezier = CubicBezier(p0, Vector2d(1.0 / 3.0, 0.0), Vector2d(2.0 / 3.0, 1.0), p1)

        assertEquals(bezier.start, hermite.bezier.start)
        // floating point error
        assert(abs((hermite.bezier.control1 - bezier.control1).x) < 0.00001)
        assert(abs((hermite.bezier.control1 - bezier.control1).y) < 0.00001)
        assert(abs((hermite.bezier.control2 - bezier.control2).x) < 0.00001)
        assert(abs((hermite.bezier.control2 - bezier.control2).y) < 0.00001)
        assertEquals(bezier.end, hermite.bezier.end)
    }
}
