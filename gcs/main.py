import sys
import logging
from PyQt5 import QtWidgets
from mavlink import MavlinkThread
from gcs_graphing import MyWin

if __name__ == "__main__":
    # logging.basicConfig(
    #     stream=sys.stdout, level=logging.INFO,
    #     format="%(asctime)-15s %(message)s"
    # )

    ######################################
    app = QtWidgets.QApplication(sys.argv)
    myapp = MyWin()

    myapp.show()
    ######################################

    thread = MavlinkThread()
    thread.new_state_record.connect(myapp.state_msg)
    thread.new_imu_isc_record.connect(myapp.imu_isc_msg)
    thread.new_imu_rsc_record.connect(myapp.imu_rsc_msg)
    thread.start()

    sys.exit(app.exec_())
