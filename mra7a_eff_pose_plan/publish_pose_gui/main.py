import sys;
#sys.path.append("/Users/lmn/kinetic_ws_rec/install_isolated/lib/python2.7/site-packages/")
from publish_pose_ui_deal import Publish_pose_ui_deal
from PyQt5.QtWidgets import QApplication, QMainWindow



if __name__ == '__main__':
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Publish_pose_ui_deal(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())