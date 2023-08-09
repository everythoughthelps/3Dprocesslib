from PIL import Image
import numpy as np
#from open3d import read_point_cloud, draw_geometries
import time

class depth2pointcloud():

    def __init__(self, rgb_file, depth_file, pointcloud_dst, cam_intrinsics, scalefactor):
        self.rgb_file = rgb_file
        self.depth_file = depth_file
        self.pc_dst = pointcloud_dst
        self.cam_intrinsics = cam_intrinsics
        self.depth_scale= scalefactor
        self.rgb = Image.open(rgb_file)
        self.depth = Image.open(depth_file).convert('I')
        self.width = self.rgb.size[0]
        self.height = self.rgb.size[1]

    def calculate(self):
        t1 = time.time()

        depth = np.asarray(self.depth, dtype=np.uint16).T
        # depth[depth==65535]=0
        self.Z = depth / self.depth_scale
        fx, fy, cx, cy = self.cam_intrinsics[0][0], \
            self.cam_intrinsics[1][1], \
            self.cam_intrinsics[0][2], \
            self.cam_intrinsics[1][2]

        X = np.zeros((self.width, self.height))
        Y = np.zeros((self.width, self.height))
        for i in range(self.width):
            X[i, :] = np.full(X.shape[1], i)

        self.X = ((X - cx) * self.Z) / fx
        for i in range(self.height):
            Y[:, i] = np.full(Y.shape[0], i)
        self.Y = ((Y - cy) * self.Z) / fy

        world_coordinate = np.zeros((6, self.width * self.height))
        world_coordinate[0] = self.X.T.reshape(-1)
        world_coordinate[1] = -self.Y.T.reshape(-1)
        world_coordinate[2] = -self.Z.T.reshape(-1)
        img = np.array(self.rgb, dtype=np.uint8)
        world_coordinate[3] = img[:, :, 0:1].reshape(-1)
        world_coordinate[4] = img[:, :, 1:2].reshape(-1)
        world_coordinate[5] = img[:, :, 2:3].reshape(-1)
        self.point_cloud_data = world_coordinate

        t2 = time.time()
        print('calcualte 3d point cloud Done.', t2 - t1)

    def write_ply(self):
        t1 = time.time()
        float_formatter = lambda x: "%.4f" % x
        points = []
        for i in self.point_cloud_data.T:
            points.append("{} {} {} {} {} {} 0\n".format
                          (float_formatter(i[0]), float_formatter(i[1]), float_formatter(i[2]),
                           int(i[3]), int(i[4]), int(i[5])))

        file = open(self.pc_dst, "w")
        file.write('''ply
        format ascii 1.0
        element vertex %d
        property float x
        property float y
        property float z
        property uchar red
        property uchar green
        property uchar blue
        property uchar alpha
        end_header
        %s
        ''' % (len(points), "".join(points)))
        file.close()

        t2 = time.time()
        print("Write into .ply file Done.", t2 - t1)

   # def show_point_cloud(self):
   #     pcd = read_point_cloud(self.pc_dst)
   #     draw_geometries([pcd])


if __name__ == '__main__':
    cam_intrinsics = np.array([[5.1885790117450188e+02, 0, 3.2558244941119034e+02],
                                    [0, 5.1946961112127485e+02, 2.5373616633400465e+02],
                                    [0,0,1]])
    rgb_file = "./00042_colors.png"
    depth_file = "./00042_depth.png"
    save_ply = "./adv_00042.ply"
    #depth = Image.open(depth_file)
    #depth = depth.crop((28, 21, 611, 458))
    #depth.save('./00042_depth.png')
    a = depth2pointcloud(rgb_file=rgb_file,
                         depth_file=depth_file,
                         pointcloud_dst=save_ply,
                         cam_intrinsics=cam_intrinsics,
                         scalefactor=1000
                         )
    a.calculate()
    a.write_ply()
    #a.show_point_cloud()