#!/usr/bin/env python

import sys
import math
from pyqtgraph.Qt import QtGui, QtCore
from numpy import arange, array, ones, linalg
import numpy as np
import pyqtgraph as pg

class SimplePlot:

    def __init__(self):
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsWindow(title="Plotter")
        self.win.resize(800,600)
        self.plot = self.win.addPlot(title="Plot")
        self.plot.enableAutoRange('y', False)
        self.plot.setYRange(-math.pow(10,7), math.pow(10,7))
        self.curve = self.plot.plot(pen='y')
        self.values = np.array([])


    def tick(self):

        # Setting the xrange to the last Xth number of elements.
        if (self.values.size > 100):
            self.plot.setXRange(self.values.size - 100, self.values.size-1)
        self.app.processEvents()

        #self.plot.setYRange(-math.pow(10,7), math.pow(10,7), update=False)
        #self.app.setXRange()

    def addPoint(self, y):
        self.values = np.append(self.values, y)
        self.curve.setData(self.values)

    def setXRange(self, xrange):
        self.plot.setXRange
