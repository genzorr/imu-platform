import sys
import logging
from PyQt5 import QtWidgets

from transfer import TransferThread
from gcs_graphing import MyWin

if __name__ == "__main__":
    # logging.basicConfig(
    #     stream=sys.stdout, level=logging.INFO,
    #     format="%(asctime)-15s %(message)s"
    # )

    ######################################
    app = QtWidgets.QApplication(sys.argv)
    myapp = MyWin()

    # self.centralwidget.setLayout(self.horizontalLayout)

    myapp.show()
    myapp.showFullScreen()
    ######################################

    thread = TransferThread()

    thread.new_state_record.connect(myapp.state_msg)
    thread.new_imu_isc_record.connect(myapp.imu_isc_msg)
    thread.new_imu_rsc_record.connect(myapp.imu_rsc_msg)
    thread.new_serial_record.connect(myapp.serial_msg)

    thread.set_status_signal.connect(myapp.set_status)
    thread.blank_status_signal.connect(myapp.blank_status)

    thread.start()

    sys.exit(app.exec_())
