import numpy as np
import math
import matplotlib.pyplot as pl
from global_map import *
import pylab as pl

ROBOT_ID_OFFSET=10


class grid_cell:
    def __init__(self):
        self.value=-1
        self.position=[0,0]
        
    def populate_cell(self, value, position):
        self.value=value
        self.position=position

class GridMap:
    def __init__(self):
        self.grid_map=[]
        self.grid2D=[]
        self.gmap=gmap
        self.max_rows=0
        self.max_cols=0
        self.rob_position=[0,0]
        self.frontiers=[]
        self.frontiers_x=[]
        self.frontiers_y=[]
        self.obstacles={}

    def generate_grid_map(self,rows, cols, num_obstacles, size_obstacles, rob_start_pos):
        self.grid_map=[]
        self.max_rows=rows
        self.max_cols=cols
        self.rob_position=rob_start_pos
        
        for iter1 in range(rows):
            for iter2 in range(cols):
                temp=grid_cell()
                temp.populate_cell(-1,[iter1, iter2] )
                self.grid_map.append(temp)   
        self.grid2D=np.zeros((rows, cols))
        #obstacles assumed to be circular
        deg = list(range(0, 360, 5))
        deg.append(0)
        #code for getting obstacles on the map

    def fit_ellipse_along_frontier(self):

        x=np.array([float(x[0]) for x in map.frontiers])
        y=np.array([float(x[1]) for x in map.frontiers])   
        N=len(x)
        pl.plot(x, y, '.')     # given points
        pl.xlim([0,100])
        pl.ylim([0,100])
        xmean, ymean = x.mean(), y.mean()
        x -= xmean
        y -= ymean
        U, S, V = np.linalg.svd(np.stack((x, y)))

        transform = np.sqrt(2/N) * U.dot(np.diag(S))   # transformation matrix
        self.ellipse_xc=xmean
        self.ellipse_yc=ymean
        self.ellipse_tform=transform


    def draw_frontier_ellipse(self, in_map_flag):
        if not in_map_flag:
            x=np.array([float(x[0]) for x in map.frontiers])
            y=np.array([float(x[1]) for x in map.frontiers])   
            N=len(x)
            pl.plot(x, y, '.')     # given points
            pl.xlim([0,100])
            pl.ylim([0,100])

            tt = np.linspace(0, 2*np.pi, 1000)
            circle = np.stack((np.cos(tt), np.sin(tt)))    # unit circle

            fit = self.transform.dot(circle) + np.array([[self.xmean], [self.ymean]])
            pl.figure()
            pl.plot(fit[0, :], fit[1, :], 'r')
            pl.show()        
        else:
            tt = np.linspace(0, 2*np.pi, 1000)
            circle = np.stack((np.cos(tt), np.sin(tt)))    # unit circle

            fit = self.transform.dot(circle) + np.array([[self.xmean], [self.ymean]])
            fit=np.ceil(fit)
            
    def flood_fill(self, x, y,old, new, min_x, min_y,max_x, max_y):
        if x < min_x or x >= max_x or y < min_y or y >= max_y:
            return
        # secondly, check if the current position equals the old value
        if self.grid2D[x][y] != old:
            return
    
        # thirdly, set the current position to the new value
        self.grid2D[x][y] = new
        # fourthly, attempt to fill the neighboring positions
        self.flood_fill(x+1, y, old, new, min_x, min_y,max_x, max_y)
        self.flood_fill(x-1, y, old, new, min_x, min_y,max_x, max_y)
        self.flood_fill(x, y+1, old, new, min_x, min_y,max_x, max_y)
        self.flood_fill(x, y-1, old, new, min_x, min_y,max_x, max_y)

    def map_fill_boundary(self, boundary_x, boundary_y, rob_id):
        for iter in range(len(boundary_x)):
            row=int(np.round(boundary_x[iter]))
            col=int(np.round(boundary_y[iter]))

            # print("gfrsessdfgj",row, col)
            self.grid2D[row][col]=ROBOT_ID_OFFSET+rob_id

        min_x=int(min(boundary_x))
        min_y=int(min(boundary_y))
        max_x=int(max(boundary_x))
        max_y=int(max(boundary_y))
        self.flood_fill(min_x, min_y, 0, ROBOT_ID_OFFSET+rob_id,min_x, min_y,max_x, max_y)    

    def get_adjacent_cells(self, source_cell):
        cur_pos=source_cell.position
        adjacent_cells=[]
        row=cur_pos[0]
        col=cur_pos[1]
        rob_index=row*(col-1)+col -1
        adjacent_cells.append(self.grid_map[rob_index-1])
        adjacent_cells.append(self.grid_map[rob_index+1])
        
        rob_index=(row-1)*(col-1)+col-1;    
        adjacent_cells.append(self.grid_map[rob_index])
        adjacent_cells.append(self.grid_map[rob_index-1])
        adjacent_cells.append(self.grid_map[rob_index+1])
        rob_index=(row+1)*(col-1)+col-1;    
        adjacent_cells.append(self.grid_map[rob_index])
        adjacent_cells.append(self.grid_map[rob_index-1])
        adjacent_cells.append(self.grid_map[rob_index+1])

        return adjacent_cells

    

    def get_sensor_boundary(self, radius):
        rob_r=self.rob_position[0]
        rob_c=self.rob_position[1]
                
        row_iter1=rob_r
        row_iter2=rob_r
        row_iter=rob_r
        stop_scan=False
        r_reached_row=False
        stop_vert_scan1=False
        stop_vert_scan2=False

        toggle_flag=1
        while not (stop_vert_scan1 and stop_vert_scan2):

            toggle_flag*=-1
            col_iter1=rob_c
            col_iter2=rob_c
            r_reached_col1=False
            r_reached_col2=False
            thickness_cnt1=0
            thickness_cnt2=0
            stop_hor_scan1=False
            stop_hor_scan2=False
            # print(row_iter)
            if row_iter1<0:
                stop_vert_scan1=True
                stop_hor_scan1=True
            if row_iter2>=self.max_rows-1:
                stop_vert_scan2=True
                stop_hor_scan2=True
                # print("limits reached!")

            while not (stop_hor_scan1 and stop_hor_scan2) :
                if not stop_hor_scan1:
                    col_iter1+=1
                    if col_iter1<=0 or col_iter1>=self.max_cols-1:
                        stop_hor_scan1=True

                    dist=math.sqrt((rob_r-row_iter)**2+(rob_c-col_iter1)**2) 
     

                    if thickness_cnt1:
                        if self.grid2D[row_iter][col_iter1]==0:
                            self.frontiers.append([row_iter, col_iter1])
                            stop_hor_scan1=True
                            self.grid2D[row_iter][col_iter1]=2
                            continue    

                    if dist> radius:
                        r_reached_col1=True
                        thickness_cnt1+=1

                    if self.grid2D[row_iter][col_iter1]==2:
                        self.frontiers.remove([row_iter,col_iter1])
                    
                    obs_temp=self.gmap.grid2D[row_iter][col_iter1]
                    if obs_temp>2:
                        # print("found obstacle ", obs_temp)
                        if obs_temp in self.obstacles:
                            self.obstacles[obs_temp].append([row_iter,col_iter1])
                        else:
                            self.obstacles[obs_temp]=[]
                            self.obstacles[obs_temp].append([row_iter,col_iter1])
                            
                        self.grid2D[row_iter][col_iter1]=3
                    else:
                        self.grid2D[row_iter][col_iter1]=1

                if not stop_hor_scan2:                    
                    col_iter2-=1
                    if col_iter2<=0 or col_iter2>=self.max_cols-1:
                        stop_hor_scan2=True
                    
                    dist=math.sqrt((rob_r-row_iter)**2+(rob_c-col_iter2)**2) 

                    if thickness_cnt2:

                        if self.grid2D[row_iter][col_iter2]==0:

                            self.frontiers.append([row_iter, col_iter2])
                            stop_hor_scan2=True    
                            self.grid2D[row_iter][col_iter2]=2
                            continue    

                    if dist> radius:
                        r_reached_col2=True
                        # print("exceeded radius C left")
                        thickness_cnt2+=1
                    
                    if self.grid2D[row_iter][col_iter2]==2:
                        self.frontiers.remove([row_iter,col_iter2])

                    obs_temp=self.gmap.grid2D[row_iter][col_iter2]
                    if obs_temp>2:
                        # print("found obstacle ", obs_temp)
                        if obs_temp in self.obstacles:
                            self.obstacles[obs_temp].append([row_iter,col_iter2])
                        else:
                            self.obstacles[obs_temp]=[]
                            self.obstacles[obs_temp].append([row_iter,col_iter2])

                        self.grid2D[row_iter][col_iter2]=3
                    else: 
                        self.grid2D[row_iter][col_iter2]=1


            #outer loop
            if toggle_flag==-1:
                row_iter1=row_iter1-1
                if not stop_vert_scan1:
                    dist=abs(rob_r-row_iter)    


                    if dist>radius:
                        stop_vert_scan1=True
                        break
                    obs_temp=self.gmap.grid2D[row_iter1][rob_c]
                    if obs_temp>2:
                        # print("found obstacle ", obs_temp)

                        if obs_temp in self.obstacles:
                            self.obstacles[obs_temp].append([row_iter1,rob_c])
                        else:
                            self.obstacles[obs_temp]=[]
                            self.obstacles[obs_temp].append([row_iter1,rob_c])

                        self.grid2D[row_iter1][rob_c]=3
                    else:
                        self.grid2D[row_iter1][rob_c]=1
                    row_iter=row_iter1
            else:
                row_iter2=row_iter2+1
                if not stop_vert_scan2:
                    dist=abs(rob_r-row_iter2)    


                    if dist>radius:
                        stop_vert_scan2=True
                        break
                    obs_temp=self.gmap.grid2D[row_iter2][rob_c]
                    if obs_temp>2:
                        # print("found obstacle 4")
                        if obs_temp in self.obstacles:
                            self.obstacles[obs_temp].append([row_iter2,rob_c])
                        else:
                            self.obstacles[obs_temp]=[]
                            self.obstacles[obs_temp].append([row_iter2,rob_c])

                        self.grid2D[row_iter2][rob_c]=3
                    else:
                            self.grid2D[row_iter2][rob_c]=1
                    row_iter=row_iter2
        
        for iter in range(len(self.frontiers)):
            self.frontiers_x.append(self.frontiers[iter][0])
            self.frontiers_y.append(self.frontiers[iter][1])
        self.cur_map_bound=[[min(self.frontiers_x), min(self.frontiers_y)],
         [max(self.frontiers_x), max(self.frontiers_y)]]

        return 

    def display_robot_map(self):
        for iter in range(len(self.frontiers)):
            self.grid2D[self.frontiers[iter][0]][self.frontiers[iter][1]]=2
        
        # pl.close()
        pl.imshow(list(self.grid2D))
        # pl.show()
        pl.pause(5)

# pl.imshow(list(map.grid2D))       
# map=GridMap()

# map.generate_grid_map(100, 100, 0,0, [50,50])   
# # pl.figure()
# # map.get_sensor_boundary(10)
# map.rob_position=[30, 40]
# map.get_sensor_boundary(10)
# map.rob_position=[20, 40]
# map.get_sensor_boundary(10)
# for iter in range(len(map.frontiers)):
#     map.grid2D[map.frontiers[iter][0]][map.frontiers[iter][1]]=2

# pl.imshow(list(map.grid2D))

# pl.show()
# print("Figure displayed!")

# print(map.obstacles)

