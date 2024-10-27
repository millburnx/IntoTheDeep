package com.millburnx.util.curves

import com.millburnx.util.Vector2d
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals

class BezierTest {
    @Test
    fun `Linear Bezier`() {
        val p1 = Vector2d(0.0, 0.0)
        val p2 = Vector2d(1.0, 1.0)
        val bezier = LinearBezier(p1, p2)
        val points = bezier.points
        assertContentEquals(listOf(p1, p2), points)
        assertEquals(p1, bezier.at(0.0))
        assertEquals(p2, bezier.at(1.0))
        assertEquals(Vector2d(0.5, 0.5), bezier.at(0.5))
        assertEquals(Vector2d(0.25, 0.25), bezier.at(0.25))
        assertEquals(Vector2d(0.75, 0.75), bezier.at(0.75))
    }

    @Test
    fun `Quadratic Bezier`() {
        val p1 = Vector2d(0.0, 0.0)
        val p2 = Vector2d(1.0, 1.0)
        val p3 = Vector2d(2.0, -1.0)
        val bezier = QuadraticBezier(p1, p2, p3)
        val points = bezier.points
        assertContentEquals(listOf(p1, p2, p3), points)
        assertEquals(p1, bezier.at(0.0))
        assertEquals(p3, bezier.at(1.0))
        assertEquals(Vector2d(1.0, 0.25), bezier.at(0.5))
        assertEquals(Vector2d(0.5, 0.3125), bezier.at(0.25))
        assertEquals(Vector2d(1.5, -0.1875), bezier.at(0.75))
    }

    @Test
    fun `Cubic Bezier`() {
        val p1 = Vector2d(0.0, 0.0)
        val p2 = Vector2d(2.0, 0.0)
        val p3 = Vector2d(-1.0, 1.0)
        val p4 = Vector2d(1.0, 1.0)
        val bezier = CubicBezier(p1, p2, p3, p4)
        val points = bezier.points
        assertContentEquals(listOf(p1, p2, p3, p4), points)
        assertEquals(p1, bezier.at(0.0))
        assertEquals(p4, bezier.at(1.0))
        assertEquals(Vector2d(0.5, 0.5), bezier.at(0.5))
        assertEquals(Vector2d(0.71875, 0.15625), bezier.at(0.25))
        assertEquals(Vector2d(0.28125, 0.84375), bezier.at(0.75))
    }
}
