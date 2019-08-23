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
        self.accel_x = []
        self.accel_y = []
        self.accel_z = []

        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []

        self.magn_x = []
        self.magn_y = []
        self.magn_z = []

        self.time = []

        self.length = 150
        self.cut = 11


        # Accel graph
        self.ui.graph_w_top = pg.GraphicsLayoutWidget(self.ui.centralwidget)
        self.ui.verticalLayout_5.addWidget(self.ui.graph_w_top)
        self.ui.plot_layout_top = pg.GraphicsLayout()
        self.ui.graph_w_top.addItem(self.ui.plot_layout_top)

        self.graph_accel = pg.PlotItem(title='Accel', labels={'left': 'accel', 'bottom': 'time'})
        self.ui.plot_layout_top.addItem(self.graph_accel)

        self.accel_x_plot = self.graph_accel.plot()
        self.accel_y_plot = self.graph_accel.plot()
        self.accel_z_plot = self.graph_accel.plot()


        # Gyro graph
        self.ui.graph_w_middle = pg.GraphicsLayoutWidget(self.ui.centralwidget)
        self.ui.verticalLayout_5.addWidget(self.ui.graph_w_middle)
        self.ui.plot_layout_middle = pg.GraphicsLayout()
        self.ui.graph_w_middle.addItem(self.ui.plot_layout_middle)

        self.graph_gyro = pg.PlotItem(title="Gyro", labels={'left': 'gyro', 'bottom': 'time'})
        self.ui.plot_layout_middle.addItem(self.graph_gyro)

        self.gyro_x_plot = self.graph_gyro.plot()
        self.gyro_y_plot = self.graph_gyro.plot()
        self.gyro_z_plot = self.graph_gyro.plot()


        # Magneto graph
        self.ui.graph_w_bottom = pg.GraphicsLayoutWidget(self.ui.centralwidget)
        self.ui.verticalLayout_5.addWidget(self.ui.graph_w_bottom)
        self.ui.plot_layout_bottom = pg.GraphicsLayout()
        self.ui.graph_w_bottom.addItem(self.ui.plot_layout_bottom)

        self.graph_magn = pg.PlotItem(title="Magnetometer", labels={'left': 'magnetometer', 'bottom': 'time'})
        self.ui.plot_layout_bottom.addItem(self.graph_magn)

        self.magn_x_plot = self.graph_magn.plot()
        self.magn_y_plot = self.graph_magn.plot()
        self.magn_z_plot = self.graph_magn.plot()


        # 3D graph
        self.ui.dockwid = QtWidgets.QDockWidget()
        self.ui.grid3DGrafLayout.addWidget(self.ui.dockwid)
        self.ui.glwid = self.plane_widget
        self.ui.dockwid.setWidget(self.ui.glwid)

        self.ui.glwid.show()

    def clear_log(self):
        self.ui.textBrowser.setText('')


    def log_add(self, log_msg):
        self.ui.textBrowser.setText(str(log_msg) + '\n')


    @QtCore.pyqtSlot(list)
    def msg(self, msgs):
        for i in range(len(msgs)):
            self.accel_x.append(msgs[i].accelData[0])
            self.accel_y.append(msgs[i].accelData[1])
            self.accel_z.append(msgs[i].accelData[2])

            self.gyro_x.append(msgs[i].gyroData[0])
            self.gyro_y.append(msgs[i].gyroData[1])
            self.gyro_z.append(msgs[i].gyroData[2])

            self.magn_x.append(msgs[i].magnData[0])
            self.magn_y.append(msgs[i].magnData[1])
            self.magn_z.append(msgs[i].magnData[2])

            self.time.append(msgs[i].time)

            quat = pyquaternion.Quaternion(msgs[i].quaternion)
            self.plane_widget._update_rotation(quat)

            # # -----------------------------------------------------------------------
            # self.accel_file = open(ACCEL_PATH, 'a')
            # self.magn_file = open(MAGN_PATH, 'a')
            # # -----------------------------------------------------------------------
            # self.accel_file.write(str(msgs[i].accelData[0]) + '\t' + str(msgs[i].accelData[1]) + '\t' + str(msgs[i].accelData[2]) + '\n')
            # self.magn_file.write(str(msgs[i].magnData[0]) + '\t' + str(msgs[i].magnData[1]) + '\t' + str(msgs[i].magnData[2]) + '\n')
            # # -----------------------------------------------------------------------
            # self.accel_file.close()
            # self.magn_file.close()


        if len(self.time) > self.length:
            self.accel_x = self.accel_x[self.cut:(self.length - 1)]
            self.accel_y = self.accel_y[self.cut:(self.length - 1)]
            self.accel_z = self.accel_z[self.cut:(self.length - 1)]

            self.gyro_x = self.gyro_x[self.cut:(self.length - 1)]
            self.gyro_y = self.gyro_y[self.cut:(self.length - 1)]
            self.gyro_z = self.gyro_z[self.cut:(self.length - 1)]

            self.magn_x = self.magn_x[self.cut:(self.length - 1)]
            self.magn_y = self.magn_y[self.cut:(self.length - 1)]
            self.magn_z = self.magn_z[self.cut:(self.length - 1)]

            self.time = self.time[self.cut:(self.length - 1)]


        self.accel_x_plot.setData(x=self.time, y=self.accel_x, pen=('r'), width=0.5)
        self.accel_y_plot.setData(x=self.time, y=self.accel_y, pen=('g'), width=0.5)
        self.accel_z_plot.setData(x=self.time, y=self.accel_z, pen=('b'), width=0.5)

        self.gyro_x_plot.setData(x=self.time, y=self.gyro_x, pen=('r'), width=0.5)
        self.gyro_y_plot.setData(x=self.time, y=self.gyro_y, pen=('g'), width=0.5)
        self.gyro_z_plot.setData(x=self.time, y=self.gyro_z, pen=('b'), width=0.5)

        self.magn_x_plot.setData(x=self.time, y=self.magn_x, pen=('r'), width=0.5)
        self.magn_y_plot.setData(x=self.time, y=self.magn_y, pen=('g'), width=0.5)
        self.magn_z_plot.setData(x=self.time, y=self.magn_z, pen=('b'), width=0.5)

    @QtCore.pyqtSlot(list)
    def imu_rsc_msg(self, msgs):
        for i in range(len(msgs)):
            self.a_RSC_x.append(msgs[i].accel[0])
            self.a_RSC_y.append(msgs[i].accel[1])
            self.a_RSC_z.append(msgs[i].accel[2])

            # self.accel_f.write("%f\t%f\t%f\n" % (msgs[i].accel[0], msgs[i].accel[1], msgs[i].accel[2]))
            # self.compass_f.write("%f\t%f\t%f\n" % (msgs[i].compass[0], msgs[i].compass[1], msgs[i].compass[2]))

            self.time_RSC.append(msgs[i].time)

            self.av_x.append(msgs[i].gyro[0])
            self.av_y.append(msgs[i].gyro[1])
            self.av_z.append(msgs[i].gyro[2])

            self.vmf_x.append(msgs[i].compass[0])
            self.vmf_y.append(msgs[i].compass[1])
            self.vmf_z.append(msgs[i].compass[2])

        if len(self.time_RSC) > self.lenght:
            self.time_RSC = self.time_RSC[self.cut:(self.lenght - 1)]
            self.av_x = self.av_x[self.cut:(self.lenght - 1)]
            self.av_y = self.av_y[self.cut:(self.lenght - 1)]
            self.av_z = self.av_z[self.cut:(self.lenght - 1)]

            self.vmf_x = self.vmf_x[self.cut:(self.lenght - 1)]
            self.vmf_y = self.vmf_y[self.cut:(self.lenght - 1)]
            self.vmf_z = self.vmf_z[self.cut:(self.lenght - 1)]

            self.a_RSC_x = self.a_RSC_x[self.cut:(self.lenght - 1)]
            self.a_RSC_y = self.a_RSC_y[self.cut:(self.lenght - 1)]
            self.a_RSC_z = self.a_RSC_z[self.cut:(self.lenght - 1)]

        self.pl_graf_top1_x.setData(x=self.time_RSC, y=self.a_RSC_x, pen=('r'), width=0.5)
        self.pl_graf_top1_y.setData(x=self.time_RSC, y=self.a_RSC_y, pen=('g'), width=0.5)
        self.pl_graf_top1_z.setData(x=self.time_RSC, y=self.a_RSC_z, pen=('b'), width=0.5)

        self.pl_graf_middle1_x.setData(x=self.time_RSC, y=self.av_x, pen=('r'), width=0.5)
        self.pl_graf_middle1_y.setData(x=self.time_RSC, y=self.av_y, pen=('g'), width=0.5)
        self.pl_graf_middle1_z.setData(x=self.time_RSC, y=self.av_z, pen=('b'), width=0.5)

        self.pl_graf_middle2_x.setData(x=self.time_RSC, y=self.vmf_x, pen=('r'), width=0.5)
        self.pl_graf_middle2_y.setData(x=self.time_RSC, y=self.vmf_y, pen=('g'), width=0.5)
        self.pl_graf_middle2_z.setData(x=self.time_RSC, y=self.vmf_z, pen=('b'), width=0.5)

    # Слот для разбора пакета imu_isc
    @QtCore.pyqtSlot(list)
    def imu_isc_msg(self, msgs):
        print('msg_imu_isc')
        for i in range(len(msgs)):
            self.quat.append(msgs[i].quaternion)

            if FILE_WRITE:
                self.buffer_imu_isc_msg.append(str(msgs[i].time) + '\t' + '\t' +
                                               str(msgs[i].accel[0]) + ' ' + str(msgs[i].accel[1]) + ' ' + str(
                    msgs[i].accel[2]) + '\t' + '\t' +
                                               str(msgs[i].compass[0]) + ' ' + str(msgs[i].compass[1]) + ' ' + str(
                    msgs[i].compass[2]) + '\t' + '\t' +
                                               str(msgs[i].quaternion[0]) + ' ' + str(
                    msgs[i].quaternion[1]) + ' ' + str(msgs[i].quaternion[2]) + ' ' + str(msgs[i].quaternion[3]) + '\n')

            quat = pyquaternion.Quaternion(msgs[i].quaternion)
            self.plane_widget._update_rotation(quat)

        if TELEM:
            self.telem_widget_quat.set_value_1(round(self.quat[len(self.quat) - 1][0], 3))
            self.telem_widget_quat.set_value_2(round(self.quat[len(self.quat) - 1][1], 3))
            self.telem_widget_quat.set_value_3(round(self.quat[len(self.quat) - 1][2], 3))
            self.telem_widget_quat.set_value_4(round(self.quat[len(self.quat) - 1][3], 3))

        if FILE_WRITE:
            self.write_to_file(self.buffer_imu_isc_msg, file_imu_isc)
            self.buffer_imu_isc_msg = []
