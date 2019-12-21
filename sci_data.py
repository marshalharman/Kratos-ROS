#!/usr/bin/env python
import rospy
import ast
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from matplotlib import style
from std_msgs.msg import String

plt.style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(3,3,1)
ax2 = fig.add_subplot(3,3,2)
ax3 = fig.add_subplot(3,3,3)
ax4 = fig.add_subplot(3,3,4)
ax5 = fig.add_subplot(3,3,5)
ax6 = fig.add_subplot(3,3,6)
ax7 = fig.add_subplot(3,3,7)

data_ax1_y = []
data_ax2_y = []
data_ax3_y = []
data_ax4_y = []
data_ax5_y = []
data_ax6_y = []
data_ax7_y = []
 
time_x = []
count = [1]

sensor_data = ""
def animate(i):
    data_list_split = sensor_data.split("@")
    data_list = []
    
    for i in data_list_split:
        data_list.append(i.split(":"))    
    print(data)

    time_x.append(dt.datetime.now().strftime('%M:%S'))

    data_ax1_y.append(data_list[0][1])
    data_ax2_y.append(data_list[1][1])
    data_ax3_y.append(data_list[2][1])
    data_ax4_y.append(data_list[3][1])
    data_ax5_y.append(data_list[4][1])
    data_ax6_y.append(data_list[5][1])
    data_ax7_y.append(data_list[6][1])

    ax1.cla()
    ax1.plot(time_x,data_ax1_y)
    ax2.cla()
    ax2.plot(time_x,data_ax2_y)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    sensor_data = data.data    
    
    ani = animation.FuncAnimation(fig, animate, interval=1000)
    plt.show()
        
def listener():
    rospy.init_node('sci_data', anonymous=True)
    rospy.Subscriber("arduino_data", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    