#file myConvexHull.py

import numpy as np
from numpy.linalg import norm

def myConvexHull(points):
    #get index of max value in points
    max_idx = np.argmax(points, axis=0)[0]
    #get index of min value in points
    min_idx = np.argmin(points, axis=0)[0]

    #divide points into two part with a line shaped by point in min_idx and max_idx
    #get index of values above the line
    index_above = partition_above(points,points[min_idx],points[max_idx])
    #get index of values above the line
    index_below = partition_above(points,points[max_idx],points[min_idx])

    #search convex hull above line
    above_convex = get_convex_hull(points,index_above,points[min_idx],points[max_idx],[min_idx,max_idx])
    #search convex hull below line
    below_convex = get_convex_hull(points,index_below,points[max_idx],points[min_idx],[max_idx,min_idx])
    
    #combine both convex hulls
    trueConvex = above_convex + below_convex

    return np.array(trueConvex)

def get_convex_hull(points,index_A,pA,pB,pLine):
    #there isnt any points on left/right side of pLine
    if (len(index_A) == 0):
        return [pLine]
    #only one point on left/right side of pLine
    #return line ab, bc
    elif (len(index_A) == 1) :
        return [[pLine[0],index_A[0]],[index_A[0],pLine[1]]]
    #still more than one points
    #search for a points farthest from line pApB
    else :
        farthest_idx = get_farthest_point(points,pA,pB,index_A)
        farthest = points[farthest_idx]


    #index of points on left side of line pA-farthestpoint
    convex_left = partition_above_2(points,pA,farthest,index_A)
    #index of points on left side of line farthestpoint,pB
    convex_right = partition_above_2(points,farthest,pB,index_A)

    #search convex hull left side of line pA-farthest recursively
    left_hull = get_convex_hull(points,convex_left,pA,farthest,[pLine[0],farthest_idx])
    #search convex hull left side of line farthest-pB recursively
    right_hull = get_convex_hull(points,convex_right,farthest,pB,[farthest_idx,pLine[1]])

    #combine the results from left_hull and right_hull
    ultimate_hull = left_hull + right_hull

    #return
    return ultimate_hull
    

#search points by ALL points in point
def partition_above(points,pA,pB):
    points_result = []
    for i in range(len(points)):

        #Determinant 
        x = pA[0]*pB[1] + points[i][0]*pA[1] + pB[0]*points[i][1] - points[i][0]*pB[1] - pB[0]*pA[1] - pA[0]*points[i][1]
        if (x > 0):
            #point[i] is above line pApB
            points_result.append(i)

    #returns indices of points in list
    return points_result

#search points only by array of index ind 
def partition_above_2(points,pA,pB,ind):
    points_result = []
    for i in ind:
        if (points_equal(points[i],pA)):
            continue
        elif (points_equal(points[i],pB)):
            continue
        else :
            #determine whether the point is above or below line pA-pB
            x = (pA[0]*pB[1]) + (points[i][0]*pA[1]) + (pB[0]*points[i][1]) - (points[i][0]*pB[1]) - (pB[0]*pA[1]) - (pA[0]*points[i][1])
            if (x > 0):
                #point[i] is above line pApB
                points_result.append(i)

    #returns indices of points in list
    return points_result

# def count_determinant(pA,pB,pX):
#     x = pA[0]*pB[1] + pX[0]*pA[1] + pB[0]*pX[1] - pX[0]*pB[1] - pB[0]*pA[1] - pA[0]*pX[1]
#     return x

def points_equal(pA,pB):
    if (pA[0] == pB[0]):
        if (pA[1] == pB[1]):
            return True
    
    return False

#get the farthest point from line pA-pB 
def get_farthest_point(points,pA,pB,idx):
    ires = 0
    dres = 0
    for i in idx:
        # d = abs((pB[0]-pA[0])*(pA[1]-points[i][1]) - (pA[0]-points[i][0])*(points[i][1]-pA[1])) / np.sqrt(np.square(pB[0]-pA[0]) + np.square(pB[1]-pA[1]))
        d = abs(np.cross(pB-pA,points[i]-pA)/norm(pB-pA))
        if (dres < d):
            ires = i
            dres = d
    return ires

# # def distances(pA,pB,pX):
# #     #x2 = pB
# #     #x1 = pA
# #     #x0 = pX
# #     # d=abs((pB[0]-pA[0])*(pA[1]-pX[1]) - (pA[0]-pX[0])*(pX[1]-pA[1])) / np.sqrt(np.square(pB[0]-pA[0]) + np.square(pB[1]-pA[1]))
# #     d=np.cross(pB-pA,pX-pA)/norm(pB-pA)
# #     return d



# # test = np.array([[0,0],[0, 2], [1, 1], [3, 5], [3, 6], [4, 3], [4, 3], [5, 3],[5,4],[10,0],[1,-3],[3,-4],[5,-5],[1,-9],[5,-9],[-3,4],[2,-2]])

# # # solution : 0,0 0,2 3,6 10,0
# # # print(ConvexHull(test))
# # man,tap = myConvexHull(test)
# points = np.array([[1.4, 0.2],
# [1.4, 0.2],
#  [1.3, 0.2],
#  [1.5, 0.2],
#  [1.4, 0.2],
#  [1.7, 0.4],
#  [1.4, 0.3],
#  [1.5, 0.2],
#  [1.4, 0.2],
#  [1.5, 0.1],
#  [1.5, 0.2],
#  [1.6, 0.2],
#  [1.4, 0.1],
#  [1.1, 0.1],
#  [1.2, 0.2],
#  [1.5, 0.4],
#  [1.3, 0.4],
#  [1.4, 0.3],
#  [1.7, 0.3],
#  [1.5, 0.3],
#  [1.7, 0.2],
#  [1.5, 0.4],
#  [1. , 0.2],
#  [1.7, 0.5],
#  [1.9, 0.2],
#  [1.6, 0.2],
#  [1.6, 0.4],
#  [1.5, 0.2],
#  [1.4, 0.2],
#  [1.6, 0.2],
#  [1.6, 0.2],
#  [1.5, 0.4],
#  [1.5, 0.1],
#  [1.4, 0.2],
#  [1.5, 0.2],
#  [1.2, 0.2],
#  [1.3, 0.2],
#  [1.4, 0.1],
#  [1.3, 0.2],
#  [1.5, 0.2],
#  [1.3, 0.3],
#  [1.3, 0.3],
#  [1.3, 0.2],
#  [1.6, 0.6],
#  [1.9, 0.4],
#  [1.4, 0.3],
#  [1.6, 0.2],
#  [1.4, 0.2],
#  [1.5, 0.2],
#  [1.4, 0.2]])

# jiwa = myConvexHull(points)
# print(jiwa)