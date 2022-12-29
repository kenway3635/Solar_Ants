import rospy
from std_msgs.msg import String

curr_t = rospy.Time.now()
last_t = 0

last_vl = 0
last_vr = 0

def calvl(data):

    rospy.loginfo(motor_lac, data.data)

def calvr(data):

    rospy.loginfo(motor_rac, data.data)

def recieve_data():

    rospy.init_node('recieve_data', anonymous=True)

    curr_vl = rospy.Subscriber("/motor_vl", 1, calvl)
    curr_vr = rospy.Subscriber("/motor_vr", 1, calvr)
    pub = rospy.Publisher('acceleration', String, queue_size=10)
    dt = curr_t - last_t

    l_ac = (curr_vl -last_vl)/dt
    r_ac = (curr_vr - last_vr)/dt

    output_str = "left acceleration : %s, right acceleation : %s " % (l_ac, r_ac) 
    pub.publish(output_str)

    last_t = curr_t
    last_vl = curr_vl
    last_vr = curr_vr

    rospy.spin


if __name__ == '__main__':
    recieve_data()
    
