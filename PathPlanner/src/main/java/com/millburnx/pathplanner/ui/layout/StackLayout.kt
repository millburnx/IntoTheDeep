package com.millburnx.pathplanner.ui.layout

import com.millburnx.utils.vec2d.Vec2dInt
import java.awt.event.ComponentAdapter
import javax.swing.JFrame
import javax.swing.JLayeredPane

class StackLayout : JLayeredPane() {
    val popoverLayer = PopoverLayer()
    val mainLayout = MainLayout()

    init {
        add(mainLayout, DEFAULT_LAYER, 1)
        add(popoverLayer, POPUP_LAYER, 0)
    }

    fun addTo(frame: JFrame) {
        frame.add(this)
        frame.addComponentListener(object : ComponentAdapter() {
            override fun componentResized(e: java.awt.event.ComponentEvent) {
                updateBounds(Vec2dInt(frame.width, frame.height))
            }
        })
        updateBounds(Vec2dInt(frame.width, frame.height))
    }

    fun updateBounds(frameSize: Vec2dInt) {
        setBounds(0, 0, frameSize.x, frameSize.y)
        components.forEach { it.setBounds(0, 0, width, height); it.revalidate() }
        revalidate()
        repaint()
//        println("$bounds, ${mainLayout.bounds} ${mainLayout.components.map { it.bounds }}")
    }
}

