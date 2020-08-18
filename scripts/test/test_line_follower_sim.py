#!/usr/bin/env python
import sys
sys.path.append("..")
import cv2
import unittest
from line_follower import Follower


class TestSum(unittest.TestCase):
    def test_left(self):
        follower = Follower()
        path = r'./test_images/left_turn.png' #'/home/jim/simple_ws/src/line_follower/scripts/test/test_images/left_turn.png'
        image = cv2.imread(path)
        direction = follower.colorthresh(image)
        print(direction)
        self.assertEqual(direction, "FL")
    
    def test_right(self):
        follower = Follower()
        path = r'./test_images/right_turn.png'
        image = cv2.imread(path)
        direction = follower.colorthresh(image)
        print(direction)
        self.assertEqual(direction, "FR")
    
    def test_straight(self):
        follower = Follower()
        path = r'./test_images/straight.png'
        image = cv2.imread(path)
        direction = follower.colorthresh(image)
        print(direction)
        self.assertEqual(direction, "F")
    
    def test_stop(self):
        follower = Follower()
        path = r'./test/test_images/stop.png'
        image = cv2.imread(path)
        direction = follower.colorthresh(image)
        print(direction)
        self.assertEqual(direction, "RT")

if __name__ == '__main__':
    unittest.main()
