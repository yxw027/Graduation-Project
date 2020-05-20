"""
CentroidTracker类利用相邻两帧中目标之间的Euclid距离，
可以对选中的目标在相邻的帧中辨识出同一目标
"""
from scipy.spatial import distance as dist
from collections import OrderedDict # 有序字典
import numpy as np

class CentroidTracker():
    # maxDisappeared：当某个目标连续maxDisappeared帧后仍未出现在画面中，就将其清除
    def __init__(self, maxDisappeared = 50):
        # 为新的目标留存的ID
        self.nextObjectID = 0
        # 以字典的形式存储目标：{ID：[x,y]}
        self.objects = OrderedDict()
        # 以字典的形式存储某个ID已经在视野中连续消失了多少帧
        self.disappeared = OrderedDict()
        self.maxDisappeared = maxDisappeared

    # 登记新出现的目标
    def register(self, centroid):
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    # 注销已经消失的目标
    def deregister(self, objectID):
        del self.objects[object]
        del self.disappeared[object]

    # 核心代码
    # 输入： rects: 传入的视野中的所有目标[ (x,y,w,h), ...]
    # 返回：类中存储的目标
    def update(self, rects):
        if len(rects) == 0:
            # 若没有目标传入，则所有目标的消失帧+1
            for ID in list(self.disappeared.keys()):
                self.disappeared[ID] += 1
                if self.disappeared[ID] > self.maxDisappeared:
                    self.deregister(ID)

            return self.objects

        # 创建len(rects)行，2列的数组；每一行是方框的中心坐标
        inputCentroids = np.zeros((len(rects), 2), dtype="int")
        for (i, (x, y, w, h)) in enumerate(rects):
            cx = int((2 * x + w) / 2.0)
            cy = int((2 * y + h) / 2.0)
            inputCentroids[i] = (cx, cy)

        if len(self.objects) == 0:
            for i in range(0, len(inputCentroids)):
                self.register(inputCentroids[i])
        else:
            objectIDs = list(self.objects.keys())
            objectCentroids = list(self.objects.values())
            # 计算每对objectCentroids中的坐标与输入进来的坐标的距离
            # a=np.array([[0,0],
            #               [1,2]])
            # b=np.array([[3,4],
            #               [1,0]])
            # dist.cdist(a,b) = [[5, 1],
            #                    [2.82842712, 2]]
            D = dist.cdist(np.array(objectCentroids), inputCentroids)
            # 根据D中元素的大小，将最小的元素所对应的的行数i排到前面
            # rows = [0,1]
            rows = D.min(axis=1).argsort()
            # 再在每一行i中找出元素最小的所对应的列数J，即为第i个目标最临近的目标
            # cols = [1,1]
            cols = D.argmin(axis=1)[rows]
            usedRows = set()
            usedCols = set()
            for (row, col) in zip(rows, cols):
                if row in usedRows or col in usedCols:
                    continue

                objectID = objectIDs[row]
                self.objects[objectID] = inputCentroids[col]
                self.disappeared[objectID] = 0

                usedRows.add(row)
                usedCols.add(col)

            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)

            # 若存储的ID数大于等于输入进来的目标数，需要检查哪些目标消失了
            if D.shape[0] >= D.shape[1]:
                for row in unusedRows:
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1

                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)

            # 否则就新登记新加入的目标
            else:
                for col in unusedCols:
                    self.register(inputCentroids[col])
        return self.objects