package com.millburnx.dashboard.canvas

import com.millburnx.dashboard.canvas.canvasOp.*;

class CanvasImpl : Canvas {
    private val operations = mutableListOf<CanvasOp>()

    override fun getOperations(): List<CanvasOp> {
        return operations
    }

    override fun clear() {
        operations.clear()
    }

    override fun setAlpha(alpha: Double): Canvas {
        operations.add(Alpha(alpha))
        return this
    }

    override fun strokeCircle(x: Double, y: Double, radius: Double): Canvas {
        operations.add(Circle(x, y, radius, stroke = true))
        return this
    }

    override fun fillCircle(x: Double, y: Double, radius: Double): Canvas {
        operations.add(Circle(x, y, radius, stroke = false))
        return this
    }

    override fun setFill(color: String): Canvas {
        operations.add(Fill(color))
        return this
    }

    override fun drawGrid(
        x: Double,
        y: Double,
        width: Double,
        height: Double,
        numTicksX: Int,
        numTicksY: Int,
        theta: Double,
        pivotX: Double,
        pivotY: Double,
        usePageFrame: Boolean,
    ): Canvas {
        operations.add(Grid(x, y, width, height, numTicksX, numTicksY, theta, pivotX, pivotY, usePageFrame))
        return this
    }

    override fun drawImage(
        path: String,
        x: Double,
        y: Double,
        width: Double,
        height: Double,
        theta: Double,
        pivotX: Double,
        pivotY: Double,
        usePageFrame: Boolean,
    ): Canvas {
        operations.add(Image(path, x, y, width, height, theta, pivotX, pivotY, usePageFrame))
        return this
    }

    override fun strokePolygon(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        operations.add(Polygon(xPoints, yPoints, stroke = true))
        return this
    }

    override fun fillPolygon(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        operations.add(Polygon(xPoints, yPoints, stroke = false))
        return this
    }

    override fun strokeRect(x: Double, y: Double, width: Double, height: Double): Canvas {
        operations.add(
            Polygon( // clockwise
                doubleArrayOf(x, x + width, x + width, x),
                doubleArrayOf(y, y, y + height, y + height),
                stroke = true
            )
        )
        return this
    }

    override fun fillRect(x: Double, y: Double, width: Double, height: Double): Canvas {
        operations.add(
            Polygon( // clockwise
                doubleArrayOf(x, x + width, x + width, x),
                doubleArrayOf(y, y, y + height, y + height),
                stroke = false
            )
        )
        return this
    }

    override fun strokePolyline(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        operations.add(Polyline(xPoints, yPoints))
        return this
    }

    override fun strokeLine(x1: Double, y1: Double, x2: Double, y2: Double): Canvas {
        operations.add(Polyline(doubleArrayOf(x1, x2), doubleArrayOf(y1, y2)))
        return this
    }

    override fun setRotation(radians: Double): Canvas {
        operations.add(Rotation(radians))
        return this
    }

    override fun setScale(scaleX: Double, scaleY: Double): Canvas {
        operations.add(Scale(scaleX, scaleY))
        return this
    }

    override fun setStroke(color: String): Canvas {
        operations.add(Stroke(color))
        return this
    }

    override fun setStrokeWidth(width: Int): Canvas {
        operations.add(StrokeWidth(width))
        return this
    }

    override fun strokeText(
        text: String,
        x: Double,
        y: Double,
        font: String,
        theta: Double,
        usePageFrame: Boolean,
    ): Canvas {
        operations.add(Text(text, x, y, font, theta, stroke = true, usePageFrame))
        return this
    }

    override fun fillText(
        text: String,
        x: Double,
        y: Double,
        font: String,
        theta: Double,
        usePageFrame: Boolean,
    ): Canvas {
        operations.add(Text(text, x, y, font, theta, stroke = false, usePageFrame))
        return this
    }

    override fun setTranslation(x: Double, y: Double): Canvas {
        operations.add(Translate(x, y))
        return this
    }
}