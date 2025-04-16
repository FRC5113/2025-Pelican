import json
import numpy as np

def get_input():
    id = int(input("ID: "))
    x = float(input("X\": "))
    y = float(input("Y\": "))
    z = float(input("Z\": "))
    roll = float(input("Roll: "))
    pitch = float(input("Pitch: "))
    yaw = float(input("Yaw: "))
    quaternion = get_quaternion_from_euler(np.radians(roll),np.radians(pitch),np.radians(yaw))

    return {
            "ID": id,
            "pose": {
                "rotation": {
                    "quaternion": {
                        "W": quaternion[3],
                        "X": quaternion[0],
                        "Y": quaternion[1],
                        "Z": quaternion[2]
                    }
                },
                "translation": {
                    "x": x / 39.3701,
                    "y": y / 39.3701,
                    "z": z / 39.3701
                }
            }
        }

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


def main():
    name = input("Name of field: ")
    width = float(input ("Field Width: "))
    length = float(input ("Field Length: "))
    num_tags = int(input("Number of Tags: "))
    tags = []

    for i in range(num_tags):
       print(f"Tag {i}")
       tag = get_input()
       tags.append(tag)

    data = {
        "field": {
            "length": length / 39.3701,
            "width": width / 39.3701
        },
        "tags": tags
    }


    with open(f"{name}.json",'w') as f:
        f.write(json.dumps(data,indent=4))


if __name__ == "__main__":
   main()

