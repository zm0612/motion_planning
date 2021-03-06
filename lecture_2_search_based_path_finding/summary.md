## 总结

### 图搜索方法

__任何图搜索方法都遵循以下过程：__

- (1).维护一个容器存储所有将要遍历的节点；
- (2).容器初始的初始化使用起始节点；
- 循环执行，移除(根据一些预定义的规则)、扩展(所有的邻居节点)以及添加(把邻居添加到容器中)操作；
- 结束循环

__广度优先搜索：__通过维护一个队列执行图的遍历操作。

__深度优先搜索：__通过维护一个堆栈执行图的遍历操作。



### 启发式搜索(Heuristic search)

不管是广度优先搜索还是深度优先搜索，它们的迭代都是没有目的性，或者说是目的性不强，它们始终按照队列或者堆栈的数据使用方式进行迭代。实际上对于迭代的方向可以加一些限制，从而增加迭代的速度。

而启发式搜索的引入就是为了解决上述问题。



__Dijkstra算法__

不同于广度优先搜索和深度优先搜索的策略，Dijkstra算法对于节点的遍历策略目的性非常强，__它总是朝着累计代价最低的节点迭代。__

它的优点是可以找到最优的路径。



### A*算法

A*算法与Dijkstra比没有什么太多的差异，只是增加了启发式函数h(n)，启发式函数会导致A*算比Dijkstra算法具有更强的贪心程度。但是会找到局部最优路径。

__☆最优A*算法满足的条件☆__

只有当理论最小启发式函数$f(n)$小于等于



### Jump Point Search

__缺点：__只能在规则的网格地图上寻路，而且图上的点或边不能带权重，也就是不能有复杂的地形，只支持平坦和障碍两种地形。

__Inferior Neighbors__（劣性邻居）：不经过当前节点，反而得到更短路径到达的节点就是劣性节点。

__Forced Neighbors__: 由于当前节点周围有障碍物，导致本来是劣性节点的点，现在必须通过当前节点才能获得最短路径。​

__Jump Point__:

- 起点或者终点
- 当前点有Forced neighbor
- parent点到current点是对角线移动，切current点经过水平和垂直移动可以到达一个Jump Point，则current点是Jump Point

> 只有是Jump Point的点，才会在搜索的过程中加入到Open List中。

__\*算法的核心思想是\*__:在路径搜索的过程中，只关心那些Jump Point，把它们加入到Open List中，从而取代A*算法的暴力添加邻居，在很大的程度上增加了搜索的速度。

