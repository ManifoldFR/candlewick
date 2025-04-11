import pinocchio as pin
import example_robot_data as erd
import zmq


PORT = 12000

ctx = zmq.Context.instance()
sock: zmq.Socket = ctx.socket(zmq.SocketType.PUSH)
sock.connect(f"tcp://127.0.0.1:{PORT}")

robot: pin.RobotWrapper = erd.load("solo12")
model = robot.model
geom_model = robot.visual_model

model_str = model.saveToString()
geom_str = geom_model.saveToString()
sock.send_multipart([model_str.encode(), geom_str.encode()])
print("Sent")
