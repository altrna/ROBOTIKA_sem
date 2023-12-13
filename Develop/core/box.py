class Box:
    def __init__(self, SE3, aruco_id):
        self.SE3 = SE3
        self.SE3.transformation[2] = 150
        self.id = aruco_id
        self.assigned_amount = 0
        self.assigned_ids = []

    def append(self, cube_id):
        self.assigned_ids.append(cube_id)
        self.assigned_amount += 1

    def __repr__(self):
        return "Box SE3: " + self.SE3.__repr__() + "\ncube IDs: " + str(self.assigned_IDs)
