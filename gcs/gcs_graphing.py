import os
from math import *
import numpy as np
import pyqtgraph as pg
from gcs_ui import *

from PyQt5 import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from stl import mesh
from itertools import chain
import pyquaternion

MESH_PATH = os.path.abspath('theplane.stl')

ACCEL_PATH = '/home/michael/stm/stm32f4-imu/accel.txt'
MAGN_PATH = '/home/michael/stm/stm32f4-imu/magn.txt'


class PlaneWidget(gl.GLViewWidget):
    def __init__(self, mesh_path, *args, **kwargs):
        super(PlaneWidget, self).__init__(*args, **kwargs)
        self.setCameraPosition(distance=15)
        self.setBackgroundColor([120, 120, 120, 0])
        g = gl.GLGridItem()
        g.scale(2, 2, 1)
        g.setSize(350, 350, 350)
        self.addItem(g)

        isc_coord = gl.GLAxisItem()
        isc_coord.setSize(350, 350, 350)
        isc_coord.translate(0, 0, 0.5)
        self.addItem(isc_coord)

        self.plane_axis = gl.GLAxisItem()
        self.plane_axis.setSize(x=300, y=300, z=300)
        self.addItem(self.plane_axis)

        verts = self._get_mesh_points(mesh_path)
        faces = np.array([(i, i+1, i+2,) for i in range(0, len(verts), 3)])
        colors = np.array([(0.0, 1.0, 0.0, 1.0,) for i in range(0, len(verts), 3)])
        self.mesh = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False, shader='shaded')
        self.addItem(self.mesh)
        self.mesh.scale(1/50, 1/50, 1/50)


    def _get_mesh_points(self, mesh_path):
        your_mesh = mesh.Mesh.from_file(mesh_path)
        points = your_mesh.points

        points = np.array(list(chain(*points)))
        i = 0
        nd_points = np.ndarray(shape=(len(points)//3, 3,))
        for i in range(0, len(points)//3):
            nd_points[i] = points[i*3: (i+1)*3]

        return nd_points

    def _transform_object(self, target, move=True, rotate=True, scale=1 / 50):
        target.resetTransform()
        target.scale(scale, scale, scale)
        if move: target.translate(0, 0, -3)
        if rotate:
            target.rotate(degrees(self.rotation.angle), self.rotation.axis[0], self.rotation.axis[1],
                          self.rotation.axis[2])

    def _update_rotation(self, record):
        quat = pyquaternion.Quaternion(record)
        self.rotation = quat
        # print(self.rotation.angle)
        self._transform_object(self.mesh)
        self._transform_object(self.plane_axis, move=False)
        # self.mesh.rotate(90, 1, 1, 1, False)
        # self.mesh.rotate(180, 0, 0, 1)
        # self.mesh.translate(0, 0, 3.5)


class MyWin(QtWidgets.QMainWindow):

    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        # self
        self.setWindowTitle('gcs')
        self.plane_widget = PlaneWidget(mesh_path=MESH_PATH, parent=self)

        # ----------------------------------------------
        # self.accel_file = open(ACCEL_PATH, 'w')
        # self.magn_file = open(MAGN_PATH, 'w')
        # self.accel_file.close()
        # self.magn_file.close()

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        # Variables
        self.accel_rsc_x = []
        self.accel_rsc_y = []
        self.accel_rsc_z = []

        self.accel_isc_x = []
        self.accel_isc_y = []
        self.accel_isc_z = []

        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []

        self.magn_x = []
        self.magn_y = []
        self.magn_z = []

        self.quaternion = []

        self.time_rsc = []
        self.time_isc = []
        self.time_state = []

        self.imu_state = 255
        self.nrf_state = 255

        self.length = 150
        self.cut = 11

        # GLV
        self.ui.glv = pg.GraphicsLayoutWidget(self.ui.centralwidget)
        self.ui.graph_layout.addWidget(self.ui.glv)

        self.plot_item_accel_rsc = pg.PlotItem(title='Accelerometer RSC')
        self.ui.glv.ci.addItem(self.plot_item_accel_rsc)

        self.ui.glv.ci.nextRow()

        self.plot_item_accel_isc = pg.PlotItem(title='Accelerometer ISC')
        self.ui.glv.ci.addItem(self.plot_item_accel_isc)

        self.ui.glv.ci.nextRow()

        self.plot_item_gyro = pg.PlotItem(title='Gyroscope')
        self.ui.glv.ci.addItem(self.plot_item_gyro)

        self.ui.glv.ci.nextRow()

        self.plot_item_magn = pg.PlotItem(title='Magnetometer')
        self.ui.glv.ci.addItem(self.plot_item_magn)


        # Text and 3D
        self.ui.dockwid = QtWidgets.QDockWidget()
        self.ui.graph_3d_layout.addWidget(self.ui.dockwid)

        self.ui.glwid = self.plane_widget
        self.ui.dockwid.setWidget(self.ui.glwid)
        self.isc_coord = gl.GLAxisItem()
        self.isc_coord.setSize(25, 25, 25)
        self.ui.glwid.show()

        # graphs
        self.accel_rsc_x_graph = self.plot_item_accel_rsc.plot()
        self.accel_rsc_y_graph = self.plot_item_accel_rsc.plot()
        self.accel_rsc_z_graph = self.plot_item_accel_rsc.plot()

        self.accel_isc_x_graph = self.plot_item_accel_isc.plot()
        self.accel_isc_y_graph = self.plot_item_accel_isc.plot()
        self.accel_isc_z_graph = self.plot_item_accel_isc.plot()

        self.gyro_x_graph = self.plot_item_gyro.plot()
        self.gyro_y_graph = self.plot_item_gyro.plot()
        self.gyro_z_graph = self.plot_item_gyro.plot()

        self.magn_x_graph = self.plot_item_magn.plot()
        self.magn_y_graph = self.plot_item_magn.plot()
        self.magn_z_graph = self.plot_item_magn.plot()


    @QtCore.pyqtSlot(list)
    def state_msg(self, msgs):
        for i in range(len(msgs)):
            tmp = msgs[i].MPU_state
            if tmp != self.imu_state:
                self.imu_state = tmp
                phrase = 'IMU state {}'.format(tmp)
                self.ui.textBrowser.setText(phrase)
            else:
                pass

    @QtCore.pyqtSlot(list)
    def imu_rsc_msg(self, msgs):
        for i in range(len(msgs)):
            self.accel_rsc_x.append(msgs[i].accel[0])
            self.accel_rsc_y.append(msgs[i].accel[1])
            self.accel_rsc_z.append(msgs[i].accel[2])

            # print('{:.2f}\t{:.2f}\t{:.2f}'.format(*msgs[i].accel))

            self.gyro_x.append(msgs[i].gyro[0])
            self.gyro_y.append(msgs[i].gyro[1])
            self.gyro_z.append(msgs[i].gyro[2])

            self.time_rsc.append(msgs[i].time)

            # self.accel_f.write("%f\t%f\t%f\n" % (msgs[i].accel[0], msgs[i].accel[1], msgs[i].accel[2]))
            # self.compass_f.write("%f\t%f\t%f\n" % (msgs[i].compass[0], msgs[i].compass[1], msgs[i].compass[2]))

        if len(self.time_rsc) > self.length:
            self.time_rsc = self.time_rsc[self.cut:(self.length - 1)]

            self.accel_rsc_x = self.accel_rsc_x[self.cut:(self.length - 1)]
            self.accel_rsc_y = self.accel_rsc_y[self.cut:(self.length - 1)]
            self.accel_rsc_z = self.accel_rsc_z[self.cut:(self.length - 1)]

            self.gyro_x = self.gyro_x[self.cut:(self.length - 1)]
            self.gyro_y = self.gyro_y[self.cut:(self.length - 1)]
            self.gyro_z = self.gyro_z[self.cut:(self.length - 1)]

        self.accel_rsc_x_graph.setData(x=self.time_rsc, y=self.accel_rsc_x, pen=('r'), width=0.5)
        self.accel_rsc_y_graph.setData(x=self.time_rsc, y=self.accel_rsc_y, pen=('g'), width=0.5)
        self.accel_rsc_z_graph.setData(x=self.time_rsc, y=self.accel_rsc_z, pen=('b'), width=0.5)

        self.gyro_x_graph.setData(x=self.time_rsc, y=self.gyro_x, pen=('r'), width=0.5)
        self.gyro_y_graph.setData(x=self.time_rsc, y=self.gyro_y, pen=('g'), width=0.5)
        self.gyro_z_graph.setData(x=self.time_rsc, y=self.gyro_z, pen=('b'), width=0.5)

    # Слот для разбора пакета imu_isc
    @QtCore.pyqtSlot(list)
    def imu_isc_msg(self, msgs):
        for i in range(len(msgs)):
            self.accel_isc_x.append(msgs[i].accel[0])
            self.accel_isc_y.append(msgs[i].accel[1])
            self.accel_isc_z.append(msgs[i].accel[2])

            self.magn_x.append(msgs[i].magn[0])
            self.magn_y.append(msgs[i].magn[1])
            self.magn_z.append(msgs[i].magn[2])

            # self.quaternion.append(msgs[i].quaternion)

            self.time_isc.append(msgs[i].time)

        if len(self.time_isc) > self.length:
            self.time_isc = self.time_isc[self.cut:(self.length - 1)]

            self.accel_isc_x = self.accel_isc_x[self.cut:(self.length - 1)]
            self.accel_isc_y = self.accel_isc_y[self.cut:(self.length - 1)]
            self.accel_isc_z = self.accel_isc_z[self.cut:(self.length - 1)]

            self.magn_x = self.magn_x[self.cut:(self.length - 1)]
            self.magn_y = self.magn_y[self.cut:(self.length - 1)]
            self.magn_z = self.magn_z[self.cut:(self.length - 1)]

            self.quaternion = self.quaternion[self.cut:(self.length - 1)]

        self.accel_isc_x_graph.setData(x=self.time_isc, y=self.accel_isc_x, pen=('r'), width=0.5)
        self.accel_isc_y_graph.setData(x=self.time_isc, y=self.accel_isc_y, pen=('g'), width=0.5)
        self.accel_isc_z_graph.setData(x=self.time_isc, y=self.accel_isc_z, pen=('b'), width=0.5)

        self.magn_x_graph.setData(x=self.time_isc, y=self.magn_x, pen=('r'), width=0.5)
        self.magn_y_graph.setData(x=self.time_isc, y=self.magn_y, pen=('g'), width=0.5)
        self.magn_z_graph.setData(x=self.time_isc, y=self.magn_z, pen=('b'), width=0.5)

        # quat = pyquaternion.Quaternion(msgs[i].quaternion)
        # self.plane_widget._update_rotation(quat)

