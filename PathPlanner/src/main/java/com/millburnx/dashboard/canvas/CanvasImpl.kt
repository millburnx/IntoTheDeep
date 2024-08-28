package com.millburnx.dashboard.canvas

import com.millburnx.dashboard.canvas.canvasOp.CanvasOp

class CanvasImpl : Canvas {
    override fun clear() {
        TODO("Not yet implemented")
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
        TODO("Not yet implemented")
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
        TODO("Not yet implemented")
    }

    override fun fillCircle(x: Double, y: Double, radius: Double): Canvas {
        TODO("Not yet implemented")
    }

    override fun fillPolygon(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        TODO("Not yet implemented")
    }

    override fun fillRect(x: Double, y: Double, width: Double, height: Double): Canvas {
        TODO("Not yet implemented")
    }

    override fun fillText(
        text: String,
        x: Double,
        y: Double,
        font: String,
        theta: Double,
        usePageFrame: Boolean,
    ): Canvas {
        TODO("Not yet implemented")
    }

    override fun getOperations(): List<CanvasOp> {
        TODO("Not yet implemented")
    }

    override fun setAlpha(alpha: Double): Canvas {
        TODO("Not yet implemented")
    }

    override fun setFill(color: String): Canvas {
        TODO("Not yet implemented")
    }

    override fun setRotation(radians: Double): Canvas {
        TODO("Not yet implemented")
    }

    override fun setScale(scaleX: Double, scaleY: Double): Canvas {
        TODO("Not yet implemented")
    }

    override fun setStroke(color: String): Canvas {
        TODO("Not yet implemented")
    }

    override fun setStrokeWidth(width: Int): Canvas {
        TODO("Not yet implemented")
    }

    override fun setTranslation(x: Double, y: Double): Canvas {
        TODO("Not yet implemented")
    }

    override fun strokeCircle(x: Double, y: Double, radius: Double): Canvas {
        TODO("Not yet implemented")
    }

    override fun strokeLine(x1: Double, y1: Double, x2: Double, y2: Double): Canvas {
        TODO("Not yet implemented")
    }

    override fun strokePolygon(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        TODO("Not yet implemented")
    }

    override fun strokePolyline(xPoints: DoubleArray, yPoints: DoubleArray): Canvas {
        TODO("Not yet implemented")
    }

    override fun strokeRect(x: Double, y: Double, width: Double, height: Double): Canvas {
        TODO("Not yet implemented")
    }

    override fun strokeText(
        text: String,
        x: Double,
        y: Double,
        font: String,
        theta: Double,
        usePageFrame: Boolean,
    ): Canvas {
        TODO("Not yet implemented")
    }
}