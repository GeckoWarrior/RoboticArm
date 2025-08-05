from tracker import Tracker
from camera_interface import Camera


camera = Camera("robot")
camera.setup()

tracker = Tracker(camera)


while True:
    camera.update()

    if not tracker.update():
        break