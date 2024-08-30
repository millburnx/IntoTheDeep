package com.millburnx.pathplanner.ui.layout

import com.millburnx.pathplanner.ui.Theme
import com.millburnx.pathplanner.ui.components.editor.EditorList
import java.awt.BorderLayout
import java.awt.Color
import javax.swing.JPanel
import javax.swing.JScrollPane
import javax.swing.border.EmptyBorder
import javax.swing.border.MatteBorder


class EditorWrapper : JPanel() {
    val editorList = EditorList()
    val editor = JPanel().apply {
        background = Color.MAGENTA
        layout = BorderLayout()
        val title = editorList.model.title
        title.border = MatteBorder(0, 0, 1, 0, Theme.borderColor)
        add(title, BorderLayout.NORTH)
        add(editorList, BorderLayout.CENTER)
    }
    val scrollPane = JScrollPane(editor).apply {
        isOpaque = true
        // the laf alr set a regular border, idk where
    }

    init {
        layout = BorderLayout()
        border = EmptyBorder(16, 24, 16, 24)
        add(scrollPane, BorderLayout.CENTER)
    }
}