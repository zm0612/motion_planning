#ifndef _JPS_UTILS_H_
#define _JPS_UTILS_H_

#include <iostream>
///Search and prune neighbors for JPS 3D
//该结构体主要是用来为JPS搜索时做数据准备，
//穷举了各种运动方向之下应该如果进行节点扩展
//以及如果判断Forced neighbor等的情况
struct JPS3DNeib {
	// for each (dx,dy,dz) these contain:
	//    ns: neighbors that are always added
	//    f1: forced neighbors to check
	//    f2: neighbors to add if f1 is forced
	int ns[27][3][26];//第一个参数27表示一个点的下一步运动方向有27种，其中3表示下一个要访问的节点相对于当前节点的增量
	                  //26表示以当前点为起点下一个要访问的点的数量最多是26个。实际上只有起点的时候才会出现26个点可以访问
	                  //对于当向右移动一步时，实际上只需要访问一个节点，此时26个参数里面有25个是无用的
	int f1[27][3][12];//f1用来存储不同运动方向情况下需要检查的障碍物相对于当前节点的增量
	int f2[27][3][12];//f2用来存储当前节点和当前运动方向之下，可能出现Force neighbor的节点相对于当前节点的增点量
	// nsz contains the number of neighbors for the four different types of moves:
	// no move (norm 0):        26 neighbors always added
	//                          0 forced neighbors to check (never happens)
	//                          0 neighbors to add if forced (never happens)
	// straight (norm 1):       1 neighbor always added
	//                          8 forced neighbors to check
	//                          8 neighbors to add if forced
	// diagonal (norm sqrt(2)): 3 neighbors always added
	//                          8 forced neighbors to check
	//                          12 neighbors to add if forced
	// diagonal (norm sqrt(3)): 7 neighbors always added
	//                          6 forced neighbors to check
	//                          12 neighbors to add if forced
	// 上面所述的原理，实际上类比于二维情况还是比较容易理解的
	static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
	JPS3DNeib();

private:
	//以下俩算法非常考验空间想象能力，多数情况下最好直接使用结论
	void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
	void FNeib( int dx, int dy, int dz, int norm1, int dev,
	    int& fx, int& fy, int& fz, int& nx, int& ny, int& nz);
};

#endif