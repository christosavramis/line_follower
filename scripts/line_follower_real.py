import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
class TestCV:
    def run(self):
        cap = cv2.VideoCapture(0)
        while True:
            _, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow("frame",frame)

            #reduce frame size 
            h,w,d = frame.shape
            frame_det = frame[int(0.8*h):h,0:w]

            #reduce frame channels from BGR to Gray
            frame_det_gray = cv2.cvtColor(frame_det, cv2.COLOR_BGR2GRAY ) 

            #filter the frame making only the black tape visible
            ret,th1 = cv2.threshold(frame_det_gray,50,255,cv2.THRESH_BINARY)

            #reverse the filter
            th1 = cv2.subtract(255, th1)

            #find the center of the black  tape
            cx,cy = findCenter(th1)
            
            #tolerance
            tol = 25
            
            #detect if there filter is empty => there is no black tape detected
            count = cv2.countNonZero(th1)

            #  o||
            if cx < w/2 - tol:
                print('FL')
            # ||o
            elif cx > w/2 + tol:
                print('FR')
            # ||
            elif count == 0:
                print('RT')
            # |o|
            else:
                print('F')
           

            #debuging 
            if cv2.waitKey(1) == ord('q'):
                break
            #draw line-follower center line
            cv2.rectangle(frame_det, (int(w/2)+tol,h), (int(w/2)-tol,0), (255, 0, 0) , 5) 

            #draw center of the black tape
            cv2.circle(frame_det,(cx,cy), 5,(0,0,255),-1)
            cv2.imshow("frame_det",frame_det)
            cv2.imshow("th1",th1)

def findCenter(frame):
    m = cv2.moments(frame)
    if m['m00'] > 0:
        cx = int(m['m10']/m['m00'])
        cy = int(m['m01']/m['m00'])
        return cx,cy
    else:
        return 0,0

if __name__ == '__main__':
	rospy.init_node('line_follower')
        testCv = TestCV()
        testCv.run()
	rospy.spin()
