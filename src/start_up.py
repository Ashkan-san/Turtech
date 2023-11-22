import paho.mqtt.client as mqtt
from threading import Lock
# import roslaunch
# import rospy
import search.see.photographer as photo

BROKER_IP, BROKER_PORT = "dashboard.lp.smsy.haw-hamburg.de", 1901


class TurtechListener:

    def __init__(self, broker_ip, broker_port):
        self.active = False
        self.lock = Lock()

        listener = mqtt.Client(client_id="Turtech_Start")
        listener.connect(broker_ip, broker_port)
        listener.subscribe("signal/falldetected")
        listener.on_message = self.on_message
        listener.subscribe("signal/cancelationfalldetected")

        self.launch = None
        print("Listener active")
        listener.loop_forever()

    def start_signal(self):
        self.lock.acquire()
        if not self.active:
            # self.launch()
            cam = photo.Photographer()
            cam.record()

            print("Turtech was activated")
            # self.active = True
        self.lock.release()

    def break_signal(self):
        self.lock.acquire()
        if self.active:
            # self.launch.shutdown()
            print("Turtech was deactivated")
            self.active = False
        self.lock.release()

    def on_message(self, client, userdata, message):
        if message.topic == "signal/falldetected":
            self.start_signal()
        elif message.topic == "signal/cancelationfalldetected":
            self.break_signal()


    # def launch(self):
    #     rospy.init_node('Turtech', anonymous=False)
    #     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #     roslaunch.configure_logging(uuid)
    #     self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["<filepath>.launch"]) # launchfile required
    #     self.launch.start()
    #     rospy.loginfo("started")


if __name__ == '__main__':
    starter = TurtechListener(BROKER_IP, BROKER_PORT)
