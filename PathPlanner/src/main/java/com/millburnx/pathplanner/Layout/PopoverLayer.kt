package com.millburnx.pathplanner.Layout

import java.awt.Color
import java.awt.Graphics
import javax.swing.JPanel

class PopoverLayer : JPanel() {
    init {
        background = Color(1.0f, 0f, 0f, 0.5f)
        isOpaque = false
    }

    override fun paintComponent(g: Graphics) {
        g.color = background
        g.fillRect(0, 0, width, height)
        super.paintComponent(g)
    }
}