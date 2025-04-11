import pinocchio as pin
import zmq


PORT = 12000

ctx = zmq.Context.instance()
sock: zmq.Socket = ctx.socket(zmq.SocketType.PUSH)
sock.connect(f"tcp://127.0.0.1:{PORT}")

model = pin.buildSampleModelHumanoidRandom()
geom_model = pin.buildSampleGeometryModelHumanoid(model)

sock.send_multipart([model.saveToString().encode(), geom_model.saveToString().encode()])
