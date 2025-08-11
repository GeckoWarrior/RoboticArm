from vision.tracker import Tracker
from vision.camera_interface import Camera

from config import Config

config = Config()

camera = Camera("laptop")
camera.setup()

tracker = Tracker(camera, config, "tennis_ball")

while True:
    camera.update()

    if not tracker.update():
        print("done")
        break