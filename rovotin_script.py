from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
import yaml
import socket
import numpy as np

HOST = "HOST_IP"
PORT = 12345 #REPLACE WITH YOUR OWN PORT
BUFFER_SIZE = 256


def yaml_to_point(target):
    target = target["target"]
    position = []
    position.append(target[1]*-1 * 240 + 50)
    position.append(target[0] * 290)
    position.append(target[2] * 220 + 50)
    return position

class RobotManipulation():
    def _init_(self):
        self.RDK = None
        self.robot = None
        self.position = []
        self.target = None

    def connect(self):
        self.RDK = Robolink()
        self.robot = self.RDK.ItemUserPick("", ITEM_TYPE_ROBOT)
        self.target = self.RDK.AddTarget("New Target", 0)

    def update_position(self, position):
        self.position = position
        pose_ref = self.robot.Pose()
        pose_ref.setPos(self.position)
        self.target.setPose(pose_ref)

    def force_position(self, position):
        lista_zzz = [0,-5,5,-10,10,-20,20]
        for i in range(len(lista_zzz)):
            for j in range(len(lista_zzz)):
                for k in range(len(lista_zzz)):
                    try:
                        self.position = [position[0] - lista_zzz[k], position[1]- lista_zzz[j], position[2]- lista_zzz[i]]
                        pose_ref = self.robot.Pose()
                        pose_ref.setPos(self.position)
                        self.target.setPose(pose_ref)
                        self.robot.MoveJ(self.target)
                        return
                    except:
                        path_file = "path_log_error"
                        log_f = open(path_file, "a")
                        log_f.write(str(self.position) + '\t')
                        log_f.write(str(k), str(j), str(i), '\n')
                        log_f.close()
                        continue        

    def move(self):
        try:
            self.robot.MoveJ(self.target)
        except Exception as e:
            print(e)




def main():
    # Start point with respect to the robot base frame
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(5)
    conn, address = s.accept()

    P_START = [0, -342, 0]
    P_END = [340, 342, 293]
    path_file = "path_log"
    log_file = open(path_file, "w")

    robot_manipulation = RobotManipulation()
    robot_manipulation.connect()

    while True:
        data = conn.recv(BUFFER_SIZE).decode()
        if not data:
            break
        try:
            yaml_obj = yaml.safe_load(data) 
            position = yaml_to_point(yaml_obj)

            log_file = open(path_file, "a")
            log_file.write(str(position) + '\n')
            log_file.close()
            
            robot_manipulation.force_position(position)

            log_file.close()
            msg = "OK"
        except Exception as e:
            print(f"Exception encountered: {e}")
            msg = "ERROR"
        conn.send(msg.encode())
    conn.close()

if __name__ == '__main__':
    main()