package com.millburnx.pathPlanner.layout

import javax.swing.JButton
import javax.swing.JPanel
import javax.swing.border.EmptyBorder

public class Navbar : JPanel() {
    public val save: JButton = JButton("Save").apply {
    }
    public val load: JButton = JButton("Load").apply {
    }
    public val clear: JButton = JButton("Clear").apply {
    }

    init {
        layout = WrapLayout()
        border = EmptyBorder(16, 24, 16, 24)
        add(save)
        add(load)
        add(clear)
    }
}