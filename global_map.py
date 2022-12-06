import numpy as np
import matplotlib.pyplot as plt
class global_map:
    def __init__(self,grid_size) -> None:
        self.grid2D=np.zeros((grid_size[0], grid_size[1]))
        self.obstacles=[]
        self.feature=[]
        self.obstacle_discrete_points=[]
        self.feature_discrete_points=[]
        self.com_params={}
    def fit_ellipse_over_frontier(self, frontiers_x, frontiers_y, ID):
        x=np.array(frontiers_x, dtype=float)
        y=np.array(frontiers_y, dtype=float)
        N=len(x)
        xmean, ymean = x.mean(), y.mean()
        # print(x)
        # print(type(xmean))
        x -= xmean
        y -= ymean
        U, S, V = np.linalg.svd(np.stack((x, y)))
        transform = np.sqrt(2/N)*U.dot(np.diag(S))   # transformation matrix
        self.com_params[ID]=[transform, xmean, ymean]
        # tt = np.linspace(0, 2*np.pi, 1000)
        # circle = np.stack((np.cos(tt), np.sin(tt)))    # unit circle
    def draw_equi_triangle(self, point, side_l):
        N=20
        temp1=point
        temp2=point
        step=side_l/N
        l1_x=[]
        l1_y=[]
        l2_x=[]
        l2_y=[]
        l3_x=[]
        l3_y=[]
        feature_p1=np.zeros((2,N+1))
        feature_p2=np.zeros((2,N+1))
        feature_p3=np.zeros((2,N+1))
        l1_end_x=point[0]+np.cos(np.pi/6)*side_l
        l1_end_y=point[1]+np.sin(np.pi/6)*side_l
        for iter in range(N+1):
            l1_x.append(point[0]+np.cos(np.pi/6)*step*iter)
            l1_y.append(point[1]+np.sin(np.pi/6)*step*iter)
            l2_x.append(point[0]+np.cos(-np.pi/6)*step*iter)
            l2_y.append(point[1]+np.sin(-np.pi/6)*step*iter)
            l3_x.append(l1_end_x+np.cos(np.pi/2)*step*iter)
            l3_y.append(l1_end_y+np.sin(-np.pi/2)*step*iter)
        # print(l1_x)
        feature_p1[0,:]=np.array(l1_x)
        feature_p1[1,:]=np.array(l1_y)
        feature_p2[0,:]=np.array(l2_x)
        feature_p2[1,:]=np.array(l2_y)
        feature_p3[0,:]=np.array(l3_x)
        feature_p3[1,:]=np.array(l3_y)
        self.feature.append([feature_p1,feature_p2, feature_p3])
        # print(self.feature)
        # plt.plot(l1_x,l1_y,'-r')
        # plt.plot(l2_x,l2_y,'-b')
        # plt.plot(l3_x, l3_y,'-k')
        # plt.show()
    #obstacle has [x, y, r] representing a circle
    def create_obstacles(self, osbstacle_list):
        self.obstacles=osbstacle_list
    def discretize_features(self):
        #discretizing obstacles
        obs_count=3
        for iter in range(len(self.obstacles)):
            x,y=self.circle_boundary(self.obstacles[iter][0], self.obstacles[iter][1], self.obstacles[iter][2])
            x=np.round(x)
            y=np.round(y)
            points=np.vstack((x,y))
            points=np.unique(points, axis=1)
            for ind in range(len(points[0,:])):
                self.grid2D[int(points[0][ind]),int(points[1][ind])]=obs_count
            obs_count+=1
        #discretizing feature
        # print("hellooo",self.feature[0][0])
        l1=np.round(self.feature[0][0])
        l2=np.round(self.feature[0][1])
        l3=np.round(self.feature[0][2])
        for ind in range(len(l1[0,:])):
            self.grid2D[int(l1[0][ind]),int(l1[1][ind])]=20
            self.grid2D[int(l2[0][ind]),int(l2[1][ind])]=20
            self.grid2D[int(l3[0][ind]),int(l3[1][ind])]=20
    def display_grid_map(self):
        plt.imshow(list(self.grid2D))
        plt.show()
    def circle_boundary(self, x, y, size):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * np.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * np.sin(np.deg2rad(d)) for d in deg]
        return np.array(xl),np.array(yl)
gmap=global_map([100,100])
g_obstacles=[[15,16,5],[20,21,2],[30,40,4]]
gmap.create_obstacles(g_obstacles)
gmap.draw_equi_triangle([40,45],5)
gmap.discretize_features()
# gmap.display_grid_map()