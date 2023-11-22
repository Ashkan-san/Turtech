from search.see.take_photo import TakePhoto
import paho.mqtt.client as mqtt
import rospy
import shutil
from search.see.copy_machine import copy_to_vm


BROKER_IP, BROKER_PORT = "dashboard.lp.smsy.haw-hamburg.de", 1901


class Photographer:
    def __init__(self, directory="search/see/projectlp", format="png"):
        self.publisher = mqtt.Client(client_id="Turtech_Output")
        self.publisher.connect(BROKER_IP, BROKER_PORT)
        rospy.init_node('Turtech_Photographer', anonymous=False)
        self.camera = TakePhoto()
        self.directory = directory
        self.obj_id = 0
        self.pic_id = 0
        self.format = format
        rospy.sleep(5)

    def record(self):
        self.camera.take_picture(f"{self.directory}/photos/photo_{self.obj_id}_{self.pic_id}.{self.format}")
        if self.pic_id == 0:
            self.mark_position()
        self.pic_id += 1
        copy_to_vm(self.directory)
        self.publisher.publish("picture/turtech", "/home/projectuser/projectlp")

    def mark_position(self):
        shutil.copyfile("search/see/dummies/dummy_map.png", f"{self.directory}/location/location_{self.obj_id}.{self.format}")

    def switch_object(self):
        self.obj_id += 1
        self.pic_id = 0


if __name__ == "__main__":
    photographer = Photographer()
    photographer.record()
