f = open("poses_new", "r")
while True:
    line = f.readline()
    if not line == "":
        if line == "pose: \n":
            f.readline()
            f.readline()
            position = []
            orientation = []

            position.append(float(f.readline().replace("\n","").replace("x:","").replace(" ", "")))
            position.append(float(f.readline().replace("\n","").replace("y:","").replace(" ", "")))
            position.append(float(f.readline().replace("\n","").replace("z:","").replace(" ", "")))
            f.readline()
            orientation.append(float(f.readline().replace("\n","").replace("x:","").replace(" ", "")))
            orientation.append(float(f.readline().replace("\n","").replace("y:","").replace(" ", "")))
            orientation.append(float(f.readline().replace("\n","").replace("z:","").replace(" ", "")))
            orientation.append(float(f.readline().replace("\n","").replace("w:","").replace(" ", "")))
            print(position)
            print(orientation)
            print("------------------------")
    else:
        break
