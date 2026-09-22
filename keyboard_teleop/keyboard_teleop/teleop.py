#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
import sys, select, termios, tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.speed = .5
        self.turn = 1

        self.msg = """
            Reading from the keyboard  and Publishing to Twist!
            ---------------------------
            Moving around:
               u    i    o
               j    k    l
               m    ,    .

            q/z : increase/decrease max speeds by 10%
            w/x : increase/decrease only linear speed by 10%
            e/c : increase/decrease only angular speed by 10%
            anything else : stop

            CTRL-C to quit
            """

        self.moveBindings = {
                'i':(1.,0.),
                'o':(1.,-1.),
                'j':(0.,1.),
                'l':(0.,-1.),
                'u':(1.,1.),
                ',':(-1.,0.),
                '.':(-1.,1.),
                'm':(-1.,-1.),
                   }

        self.speedBindings={
                'q':(1.1,1.1),
                'z':(.9,.9),
                'w':(1.1,1.),
                'x':(.9,1.),
                'e':(1.,1.1),
                'c':(1.,.9),
                  }
        self.pub = self.create_publisher(Twist,'/vrep/twistCommand',1)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self,speed,turn):
        return "currently:\tspeed %s\tturn %s " % (self.speed,self.turn)

    def run(self):
        print(self.msg)
        self.settings = termios.tcgetattr(sys.stdin)
    


        x = 0
        th = 0
        status = 0

        while(rclpy.ok()):
            rclpy.spin_once(self,timeout_sec=0.005)
            key = self.getKey()
            if key in self.moveBindings.keys():
                x = self.moveBindings[key][0]
                th = self.moveBindings[key][1]
            elif key in self.speedBindings.keys():
                speed = speed * self.speedBindings[key][0]
                turn = turn * self.speedBindings[key][1]

                print(self.vels(self.speed,self.turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.
                th = 0.
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*self.speed
            twist.linear.y = 0.
            twist.linear.z = 0.
            twist.angular.x = 0.
            twist.angular.y = 0.
            twist.angular.z = th*self.turn
            self.pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)

    keyboard_teleop = KeyboardTeleop()

    #rclpy.spin(keyboard_teleop)
    keyboard_teleop.run()


    keyboard_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
