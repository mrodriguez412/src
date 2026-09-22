#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Joy
import sys, select, termios, tty
import curses, time

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.speed = .5
        self.turn = 1
        self.scale = 1

        self.msg = """
            Reading from the keyboard  and Publishing to Joy!
            ---------------------------
            Buttons: 0 1 2 3 4 5 6 7 8 9 (top ones, not the numerical keyboard).

            Left Thumb:           Right Thumb:
               q    w    e          u   i   o
               a    s    d          j   k   l
               z    x    c          m   ,   .

            t : Increase the scale by 10%
            g : Set the scale to 0.5
            b : Decrease the scale by 10%

            Escape to quit
            """

        self.buttonBindings = {
                '0': 0,
                '1': 1,
                '2': 2,
                '3': 3,
                '4': 4,
                '5': 5,
                '6': 6,
                '7': 7,
                '8': 8,
                '9': 9
            }

        self.leftThumbBindings = {
                'q':(-1.,1.),
                'w':(0.,1.),
                'e':(1.,1.),
                'a':(-1.,0.),
                's':(0.,0.),
                'd':(1.,0.),
                'z':(-1.,-1.),
                'x':(0.,-1.),
                'c':(1.,-1.)
                   }
        self.rightThumbBindings = {
                'u':(-1.,1.),
                'i':(0.,1.),
                'o':(1.,1.),
                'j':(-1.,0.),
                'k':(0.,0.),
                'l':(1.,0.),
                'm':(-1.,-1.),
                ',':(0.,-1.),
                '.':(1.,-1.)
                   }

        self.speedBindings={
                't':1.1,
                'b':.9,
                'g':1.
                  }

        print(self.msg)
        self.settings = termios.tcgetattr(sys.stdin)
        self.pub = self.create_publisher(Joy,'/joy',1)
        self.timer = self.create_timer(0.05, self.timer_cb)


    def curse_get(self,stdscr):
        """checking for keypress"""
        stdscr.nodelay(True)  # do not wait for input when calling getch
        return stdscr.getch()

    def getKey(self):
        key = curses.wrapper(self.curse_get)
        # if (key>=0):
        #     print("Key: "+str(key)+" "+chr(key))

        # tty.setraw(sys.stdin.fileno())
        # select.select([sys.stdin], [], [], 0.1)
        # key = sys.stdin.read(1)
        # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key



    def vels(self,speed,turn):
        return "currently:\tspeed %s\tturn %s " % (self.speed,self.turn)

    def timer_cb(self):
        joy = Joy()
        joy.axes = [0.] * 10
        joy.header.stamp = self.get_clock().now().to_msg()

        joy.header.frame_id = "n/a"
        joy.buttons = [0] * 10


        key = self.getKey()
        if key>0:
            key = chr(key)
        else:
            key = chr(0)
        if key in self.leftThumbBindings.keys():
            joy.axes[0] = self.leftThumbBindings[key][0] * self.scale
            joy.axes[1] = self.leftThumbBindings[key][1] * self.scale
        elif key in self.rightThumbBindings.keys():
            joy.axes[2] = self.rightThumbBindings[key][0] * self.scale
            joy.axes[3] = self.rightThumbBindings[key][1] * self.scale
        elif key in self.buttonBindings.keys():
            joy.buttons[self.buttonBindings[key]] = 1 
        elif key in self.speedBindings.keys():
            if self.speedBindings[key] == 1:
                self.scale = 0.5
            else:
                self.scale = self.scale * self.speedBindings[key]
                self.scale = min(1.0,max(-1.0,self.scale))
            print("scale = %f" % self.scale)
        elif (ord(key) == 27):
            raise SystemExit
        elif (key == '\x03'):
            raise SystemExit

        self.pub.publish(joy)

    def exit(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)

    keyboard_teleop = KeyboardTeleop()

    try:
        rclpy.spin(keyboard_teleop)
    except SystemExit:
        pass

    keyboard_teleop.exit()
    keyboard_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
