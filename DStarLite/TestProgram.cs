using System;
using System.Collections.Generic;
using System.Linq;

namespace DStarLiteAlgorithm
{
    /// <summary>
    /// 带权有向图的 D* Lite 算法实现
    /// </summary>
    public class DStarLite
    {
        #region 内部类定义

        /// <summary>
        /// 图节点类
        /// </summary>
        public class Node : IComparable<Node>
        {
            public int Id { get; set; }
            public double G { get; set; } = double.PositiveInfinity;
            public double Rhs { get; set; } = double.PositiveInfinity;
            public double Key1 { get; set; }
            public double Key2 { get; set; }
            public List<Edge> OutEdges { get; set; } = new List<Edge>();
            public List<Edge> InEdges { get; set; } = new List<Edge>();

            public int CompareTo(Node other)
            {
                if (Key1 < other.Key1) return -1;
                if (Key1 > other.Key1) return 1;
                if (Key2 < other.Key2) return -1;
                if (Key2 > other.Key2) return 1;
                return 0;
            }

            public override bool Equals(object obj)
            {
                return obj is Node node && Id == node.Id;
            }

            public override int GetHashCode()
            {
                return Id.GetHashCode();
            }

            public override string ToString()
            {
                return $"Node {Id} (g={G}, rhs={Rhs})";
            }
        }

        /// <summary>
        /// 边类
        /// </summary>
        public class Edge
        {
            public Node From { get; set; }
            public Node To { get; set; }
            public double Cost { get; set; }

            public Edge(Node from, Node to, double cost)
            {
                From = from;
                To = to;
                Cost = cost;
            }

            public override string ToString()
            {
                return $"{From.Id} -> {To.Id} (cost: {Cost})";
            }
        }

        /// <summary>
        /// 优先队列实现
        /// </summary>
        public class PriorityQueue<T> where T : IComparable<T>
        {
            private List<T> data;
            private Dictionary<T, int> itemToIndex;

            public PriorityQueue()
            {
                data = new List<T>();
                itemToIndex = new Dictionary<T, int>();
            }

            public void Enqueue(T item)
            {
                data.Add(item);
                itemToIndex[item] = data.Count - 1;
                int ci = data.Count - 1;
                while (ci > 0)
                {
                    int pi = (ci - 1) / 2;
                    if (data[ci].CompareTo(data[pi]) >= 0)
                        break;
                    Swap(ci, pi);
                    ci = pi;
                }
            }

            public T Dequeue()
            {
                if (data.Count == 0)
                    throw new InvalidOperationException("Queue is empty");

                T frontItem = data[0];
                itemToIndex.Remove(frontItem);

                int li = data.Count - 1;
                data[0] = data[li];
                data.RemoveAt(li);

                if (data.Count > 0)
                {
                    itemToIndex[data[0]] = 0;
                    Heapify(0);
                }

                return frontItem;
            }

            public T Peek()
            {
                if (data.Count == 0)
                    throw new InvalidOperationException("Queue is empty");
                return data[0];
            }

            public bool Contains(T item)
            {
                return itemToIndex.ContainsKey(item);
            }

            public void Remove(T item)
            {
                if (!itemToIndex.ContainsKey(item))
                    return;

                int index = itemToIndex[item];
                itemToIndex.Remove(item);

                int li = data.Count - 1;
                if (index == li)
                {
                    data.RemoveAt(li);
                    return;
                }

                data[index] = data[li];
                data.RemoveAt(li);

                if (index < data.Count)
                {
                    itemToIndex[data[index]] = index;
                    Heapify(index);
                }
            }

            public void Update(T item)
            {
                if (!itemToIndex.ContainsKey(item))
                    return;

                int index = itemToIndex[item];
                Heapify(index);

                while (index > 0)
                {
                    int parent = (index - 1) / 2;
                    if (data[index].CompareTo(data[parent]) >= 0)
                        break;
                    Swap(index, parent);
                    index = parent;
                }
            }

            private void Heapify(int i)
            {
                int smallest = i;
                int left = 2 * i + 1;
                int right = 2 * i + 2;

                if (left < data.Count && data[left].CompareTo(data[smallest]) < 0)
                    smallest = left;

                if (right < data.Count && data[right].CompareTo(data[smallest]) < 0)
                    smallest = right;

                if (smallest != i)
                {
                    Swap(i, smallest);
                    Heapify(smallest);
                }
            }

            private void Swap(int i, int j)
            {
                T tmp = data[i];
                data[i] = data[j];
                data[j] = tmp;

                itemToIndex[data[i]] = i;
                itemToIndex[data[j]] = j;
            }

            public int Count => data.Count;

            public bool IsEmpty => data.Count == 0;
        }

        #endregion

        #region 字段和属性

        private Dictionary<int, Node> nodes = new Dictionary<int, Node>();
        private PriorityQueue<Node> openList = new PriorityQueue<Node>();
        private Node start;
        private Node goal;
        private Dictionary<string, double> originalEdgeCosts = new Dictionary<string, double>();
        private Dictionary<string, double> currentEdgeCosts = new Dictionary<string, double>();
        private double km = 0; // 用于调整启发式值
        private Func<Node, Node, double> heuristicFunction;

        #endregion

        #region 公共方法

        /// <summary>
        /// 构造函数
        /// </summary>
        public DStarLite(Func<Node, Node, double> heuristic = null)
        {
            heuristicFunction = heuristic ?? DefaultHeuristic;
        }

        /// <summary>
        /// 添加节点到图中
        /// </summary>
        public void AddNode(int id)
        {
            if (!nodes.ContainsKey(id))
            {
                nodes[id] = new Node { Id = id };
            }
        }

        /// <summary>
        /// 添加边到图中
        /// </summary>
        public void AddEdge(int fromId, int toId, double cost)
        {
            if (!nodes.ContainsKey(fromId)) AddNode(fromId);
            if (!nodes.ContainsKey(toId)) AddNode(toId);

            Node from = nodes[fromId];
            Node to = nodes[toId];

            // 检查是否已存在相同的边
            var existingEdge = from.OutEdges.FirstOrDefault(e => e.To.Id == toId);
            if (existingEdge != null)
            {
                existingEdge.Cost = cost;
            }
            else
            {
                var edge = new Edge(from, to, cost);
                from.OutEdges.Add(edge);
                to.InEdges.Add(edge);
            }

            // 保存原始成本和当前成本
            string edgeKey = $"{fromId}-{toId}";
            originalEdgeCosts[edgeKey] = cost;
            currentEdgeCosts[edgeKey] = cost;
        }

        /// <summary>
        /// 初始化 D* Lite 算法
        /// </summary>
        public void Initialize(int startId, int goalId)
        {
            if (!nodes.ContainsKey(startId)) throw new ArgumentException("Start node not found");
            if (!nodes.ContainsKey(goalId)) throw new ArgumentException("Goal node not found");

            start = nodes[startId];
            goal = nodes[goalId];
            km = 0;

            // 重置所有节点
            foreach (var node in nodes.Values)
            {
                node.G = double.PositiveInfinity;
                node.Rhs = double.PositiveInfinity;
            }

            // 初始化目标节点
            goal.Rhs = 0;
            CalculateKey(goal);

            // 清空开放列表并添加目标节点
            openList = new PriorityQueue<Node>();
            openList.Enqueue(goal);
        }

        /// <summary>
        /// 计算最短路径
        /// </summary>
        public bool ComputeShortestPath()
        {
            while (!openList.IsEmpty &&
                  (CompareKeys(openList.Peek(), start) < 0 || Math.Abs(start.Rhs - start.G) > 1e-10))
            {
                Node current = openList.Dequeue();

                double[] currentKey = CalculateKey(current);
                if (CompareKeys(currentKey, CalculateKey(current)) < 0)
                {
                    // 键值已变化，重新插入
                    openList.Enqueue(current);
                }
                else if (current.G > current.Rhs)
                {
                    current.G = current.Rhs;
                    foreach (var edge in current.InEdges)
                    {
                        UpdateVertex(edge.From);
                    }
                }
                else
                {
                    current.G = double.PositiveInfinity;
                    UpdateVertex(current);
                    foreach (var edge in current.InEdges)
                    {
                        UpdateVertex(edge.From);
                    }
                }
            }

            return Math.Abs(start.Rhs - double.PositiveInfinity) > 1e-10;
        }

        /// <summary>
        /// 更新起点并重新规划
        /// </summary>
        public bool UpdateStartAndReplan(int newStartId)
        {
            if (!nodes.ContainsKey(newStartId))
                return false;

            // 更新km值
            km += heuristicFunction(start, nodes[newStartId]);

            // 更新起点
            start = nodes[newStartId];

            // 重新计算路径
            return ComputeShortestPath();
        }

        /// <summary>
        /// 获取从起点到目标点的路径
        /// </summary>
        public List<int> GetPath()
        {
            if (Math.Abs(start.Rhs - double.PositiveInfinity) < 1e-10)
                return null;

            var path = new List<int>();
            Node current = start;
            path.Add(current.Id);

            while (current != goal)
            {
                Node next = null;
                double minCost = double.PositiveInfinity;

                foreach (var edge in current.OutEdges)
                {
                    double cost = edge.Cost + edge.To.G;
                    if (cost < minCost)
                    {
                        minCost = cost;
                        next = edge.To;
                    }
                }

                if (next == null)
                    return null;

                current = next;
                path.Add(current.Id);

                // 防止无限循环
                if (path.Count > nodes.Count * 2)
                    return null;
            }

            return path;
        }

        /// <summary>
        /// 获取从起点到目标点的路径成本
        /// </summary>
        public double GetPathCost()
        {
            if (Math.Abs(start.Rhs - double.PositiveInfinity) < 1e-10)
                return double.PositiveInfinity;

            return start.Rhs;
        }

        /// <summary>
        /// 更新边的成本并重新规划
        /// </summary>
        public void UpdateEdgeCost(int fromId, int toId, double newCost)
        {
            if (!nodes.ContainsKey(fromId) || !nodes.ContainsKey(toId))
                return;

            Node from = nodes[fromId];
            var edge = from.OutEdges.FirstOrDefault(e => e.To.Id == toId);

            if (edge != null)
            {
                string edgeKey = $"{fromId}-{toId}";
                currentEdgeCosts[edgeKey] = newCost;
                edge.Cost = newCost;

                // 更新受影响的顶点
                UpdateVertex(from);

                // 重新计算路径
                ComputeShortestPath();
            }
        }

        /// <summary>
        /// 恢复边的原始成本
        /// </summary>
        public void RestoreEdgeCost(int fromId, int toId)
        {
            string edgeKey = $"{fromId}-{toId}";
            if (originalEdgeCosts.ContainsKey(edgeKey))
            {
                UpdateEdgeCost(fromId, toId, originalEdgeCosts[edgeKey]);
            }
        }

        /// <summary>
        /// 获取当前所有边的状态
        /// </summary>
        public Dictionary<string, double> GetEdgeStates()
        {
            return new Dictionary<string, double>(currentEdgeCosts);
        }

        #endregion

        #region 私有方法

        private double[] CalculateKey(Node node)
        {
            double minValue = Math.Min(node.G, node.Rhs);
            return new double[] {
                minValue + heuristicFunction(node, start) + km,
                minValue
            };
        }

        private int CompareKeys(Node node1, Node node2)
        {
            return CompareKeys(CalculateKey(node1), CalculateKey(node2));
        }

        private int CompareKeys(double[] key1, double[] key2)
        {
            if (key1[0] < key2[0]) return -1;
            if (key1[0] > key2[0]) return 1;
            if (key1[1] < key2[1]) return -1;
            if (key1[1] > key2[1]) return 1;
            return 0;
        }

        private double DefaultHeuristic(Node a, Node b)
        {
            // 默认启发式函数 - 可根据实际需求重写
            return Math.Abs(a.Id - b.Id); // 简单使用节点ID差的绝对值
        }

        private void UpdateVertex(Node node)
        {
            if (node != goal)
            {
                node.Rhs = double.PositiveInfinity;
                foreach (var edge in node.OutEdges)
                {
                    double cost = edge.Cost + edge.To.G;
                    if (cost < node.Rhs)
                    {
                        node.Rhs = cost;
                    }
                }
            }

            if (openList.Contains(node))
            {
                openList.Remove(node);
            }

            if (Math.Abs(node.G - node.Rhs) > 1e-10)
            {
                double[] key = CalculateKey(node);
                node.Key1 = key[0];
                node.Key2 = key[1];
                openList.Enqueue(node);
            }
        }

        #endregion
    }

    #region 示例和使用代码

    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("D* Lite 算法示例");

            // 创建 D* Lite 实例
            DStarLite dstar = new DStarLite();

            // 添加节点和边 - 创建一个简单的有向图
            dstar.AddEdge(1, 2, 1.0);
            dstar.AddEdge(1, 3, 5.0);
            dstar.AddEdge(2, 3, 2.0);
            dstar.AddEdge(2, 4, 4.0);
            dstar.AddEdge(3, 4, 1.0);
            dstar.AddEdge(4, 5, 3.0);
            dstar.AddEdge(3, 5, 6.0);
            dstar.AddEdge(5, 6, 2.0);
            dstar.AddEdge(4, 6, 4.0);

            // 初始化算法
            dstar.Initialize(1, 6);

            // 计算初始路径
            if (dstar.ComputeShortestPath())
            {
                var path = dstar.GetPath();
                double cost = dstar.GetPathCost();
                Console.WriteLine($"初始路径: {string.Join(" -> ", path)}, 成本: {cost}");
            }
            else
            {
                Console.WriteLine("未找到路径");
            }

            // 模拟起点变化
            Console.WriteLine("\n更新起点到节点 2");
            if (dstar.UpdateStartAndReplan(2))
            {
                var path = dstar.GetPath();
                double cost = dstar.GetPathCost();
                Console.WriteLine($"新路径: {string.Join(" -> ", path)}, 成本: {cost}");
            }
            else
            {
                Console.WriteLine("未找到路径");
            }

            // 模拟边成本变化
            Console.WriteLine("\n更新边 3->4 的成本为 10.0");
            dstar.UpdateEdgeCost(3, 5, 10.0);

            if (dstar.ComputeShortestPath())
            {
                var path = dstar.GetPath();
                double cost = dstar.GetPathCost();
                Console.WriteLine($"更新后的路径: {string.Join(" -> ", path)}, 成本: {cost}");
            }
            else
            {
                Console.WriteLine("未找到路径");
            }

            // 再次更新起点
            Console.WriteLine("\n更新起点到节点 3");
            if (dstar.UpdateStartAndReplan(3))
            {
                var path = dstar.GetPath();
                double cost = dstar.GetPathCost();
                Console.WriteLine($"新路径: {string.Join(" -> ", path)}, 成本: {cost}");
            }
            else
            {
                Console.WriteLine("未找到路径");
            }

            // 恢复原始成本
            Console.WriteLine("\n恢复边 3->4 的原始成本");
            dstar.RestoreEdgeCost(3, 4);

            if (dstar.ComputeShortestPath())
            {
                var path = dstar.GetPath();
                double cost = dstar.GetPathCost();
                Console.WriteLine($"恢复后的路径: {string.Join(" -> ", path)}, 成本: {cost}");
            }
            else
            {
                Console.WriteLine("未找到路径");
            }
        }
    }

    #endregion
}