#import sys;
#sys.path.append("/Users/lmn/kinetic_ws_rec/install_isolated/lib/python2.7/site-packages/")
import rospy
from publish_pose_gui import Ui_MainWindow

from std_msgs.msg._Float32MultiArray import Float32MultiArray


class Publish_pose_ui_deal():
    ui_mainwindow = Ui_MainWindow()
    def __init__(self, MainWindow):
        self.ui_mainwindow.setupUi(MainWindow)
        self.pub = rospy.Publisher('end_effector_pose', Float32MultiArray, queue_size=10)
        rospy.init_node('publish_pose_node', anonymous=True)
        self.pose = Float32MultiArray()

        self.ui_mainwindow.pushButton.clicked.connect(self.publish_pose)

    def publish_pose(self):
        self.pose.data = []
        self.pose.data.insert(0, float(self.ui_mainwindow.le_x.text().__str__()))
        self.pose.data.insert(1, float(self.ui_mainwindow.le_y.text().__str__()))
        self.pose.data.insert(2, float(self.ui_mainwindow.le_z.text().__str__()))
        self.pose.data.insert(3, float(self.ui_mainwindow.le_R.text().__str__())*3.14/180)
        self.pose.data.insert(4, float(self.ui_mainwindow.le_P.text().__str__())*3.14/180)
        self.pose.data.insert(5, float(self.ui_mainwindow.le_Y.text().__str__())*3.14/180)
        self.pub.publish(self.pose)


