import numpy as np
import matplotlib.pyplot as plt

class global_map:
    def __init__(self,grid_size) -> None:
        self.grid2D=np.zeros((grid_size[0], grid_size[1]))
        self.obstacles=[]
        self.feature=[]
            
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
        
        l1_end_x=point[0]+np.cos(np.pi/6)*side_l
        l1_end_y=point[1]+np.sin(np.pi/6)*side_l

        for iter in range(N+1):
            l1_x.append(point[0]+np.cos(np.pi/6)*step*iter)
            l1_y.append(point[1]+np.sin(np.pi/6)*step*iter)
            l2_x.append(point[0]+np.cos(-np.pi/6)*step*iter)
            l2_y.append(point[1]+np.sin(-np.pi/6)*step*iter)
            l3_x.append(l1_end_x+np.cos(np.pi/2)*step*iter)
            l3_y.append(l1_end_y+np.sin(-np.pi/2)*step*iter)

        self.feature.append([l1_x,l1_y],[l2_x,l2_y],[l3_x,l3_y])
        
        plt.plot(l1_x,l1_y,'-r')
        plt.plot(l2_x,l2_y,'-b')
        plt.plot(l3_x, l3_y,'-k')
        plt.show()

        def discretize_features():
            pass

draw_equi_triangle([2,4],3)
