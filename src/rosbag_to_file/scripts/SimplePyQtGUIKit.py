#!/usr/bin/env python
# -*- coding:utf-8 -*-

from PyQt4 import QtGui
from PyQt4 import QtCore
import sys

class SimplePyQtGUIKit:
    def QuitApp(self):
        QtCore.QApplication.quit()

    @classmethod
    def GetFilePath(self, caption="Open File", filefilter=""):
        u"""
            "Images (*.png *.xpm *.jpg);;Text files (*.txt);;XML files (*.xml)"
        """

        files = QtGui.QFileDialog.getOpenFileNames(caption=caption, filter=filefilter)
        return [str(f) for f in files]

    @classmethod
    def GetDirectory(self, caption="Open Directory", directory=".", filefilter=""):
        u"""
            "Images (*.png *.xpm *.jpg);;Text files (*.txt);;XML files (*.xml)"
        """

        dialog = QtGui.QFileDialog(caption=caption, directory=directory, filter=filefilter)
        dialog.setFileMode(QtGui.QFileDialog.DirectoryOnly)
        if dialog.exec_() == QtGui.QDialog.Accepted:
            dirs = dialog.selectedFiles()
        else:
            dirs = []
        
        return [str(d) for d in dirs]

    @classmethod
    def GetCheckButtonSelect(self, select_list, title="Select", msg="",app=None):
        """
        Get selected check button options

        title: Window name
        mag: Label of the check button
        return selected dictionary
            {'sample b': False, 'sample c': False, 'sample a': False}
        """
 
        if app is None:
          app = QtCore.QApplication(sys.argv)
        win = QtGui.QWidget()

        scrollArea = QtGui.QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollAreaWidgetContents = QtGui.QWidget(scrollArea)
        scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 380, 247))
        scrollArea.setWidget(scrollAreaWidgetContents)
        
        layout = QtGui.QGridLayout()
        verticalLayoutScroll = QtGui.QVBoxLayout(scrollAreaWidgetContents)
        layoutIndex = 0

        if msg is not "":
            label = QtGui.QLabel(msg)
            layout.addWidget(label,layoutIndex,0)
            layoutIndex = layoutIndex+1

        checkboxes = []
        select_list.append("All topics")
        for select in select_list:
            checkbox = QtGui.QCheckBox(select)
            verticalLayoutScroll.addWidget(checkbox)
            layoutIndex = layoutIndex+1
            checkboxes.append(checkbox)

        layout.addWidget(scrollArea)
        btn = QtGui.QPushButton("OK")
        btn.clicked.connect(app.quit)
        layout.addWidget(btn,layoutIndex,0)
        layoutIndex = layoutIndex+1

        win.setLayout(layout)
        win.setWindowTitle(title)
        win.show()
        app.exec_()

        result = {}
        if checkboxes[-1].isChecked():  # if "All topics" is checked, all topics
            for selection in select_list:
                result[selection] = True
        else:
            for index in range(len(select_list[:-1])):  # check all boxes except the "All topics" box
                checkbox = checkboxes[index]
                selection = select_list[index]
                result[selection] = checkbox.isChecked()

        return result
