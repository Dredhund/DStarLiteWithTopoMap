using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathPlanningCS.Dstar
{
    public class Node
    {
        public int Id { get; }
        public List<Edge> OutEdges { get; } = new();   // 出边
        public List<Edge> InEdges { get; } = new();   // 入边（用于 Pred）
        public Node(int id) => Id = id;
        public override string ToString() => $"N{Id}";
    }

    public class Edge(Node from, Node to, double cost)
    {
        public Node From { get; } = from;
        public Node To { get; } = to;
        public double Cost { get; set; } = cost;      // 可变
    }
    internal sealed class PrioQueue<T, TP> where TP : IComparable<TP>
    {
        private readonly List<(T item, TP prio)> _h = new();
        public int Count => _h.Count;

        public void Clear() => _h.Clear();
        public void Enqueue(T it, TP p) { _h.Add((it, p)); Up(_h.Count - 1); }
        public T Dequeue()
        {
            var res = _h[0].item;
            var last = _h[^1];
            _h.RemoveAt(_h.Count - 1);
            if (_h.Count > 0) { _h[0] = last; Down(0); }
            return res;
        }
        public T Peek() => _h[0].item;
        public TP PeekPrio() => _h[0].prio;
        public bool Remove(T it)
        {
            int i = _h.FindIndex(x => Equals(x.item, it));
            if (i < 0) return false;
            var last = _h[^1];
            _h.RemoveAt(_h.Count - 1);
            if (i < _h.Count) { _h[i] = last; Up(i); Down(i); }
            return true;
        }
        private void Up(int i)
        {
            while (i > 0)
            {
                int p = (i - 1) / 2;
                if (_h[p].prio.CompareTo(_h[i].prio) <= 0) break;
                (_h[i], _h[p]) = (_h[p], _h[i]); i = p;
            }
        }
        private void Down(int i)
        {
            while (true)
            {
                int l = 2 * i + 1, r = l + 1, s = i;
                if (l < _h.Count && _h[l].prio.CompareTo(_h[s].prio) < 0) s = l;
                if (r < _h.Count && _h[r].prio.CompareTo(_h[s].prio) < 0) s = r;
                if (s == i) break;
                (_h[i], _h[s]) = (_h[s], _h[i]); i = s;
            }
        }
    }
    public enum Tag { NEW, OPEN, CLOSED }

    public class DStarDirected
    {
        private readonly Dictionary<Node, (double h, Tag tag, Node parent)> _state = new();
        private readonly PrioQueue<Node, double> _open = new();
        private Node _start, _goal;

        public DStarDirected(Node goal)
        {
            _goal = goal;
            _state.Clear(); _open.Clear();
            _state[goal] = (0, Tag.NEW, null);
            Insert(goal, 0);
        }

        public void SetStartNode(Node start)
        {
            _start = start;
        }

        public List<Node> ComputePath()
        {
            while (_open.Count > 0 &&
                   (_open.PeekPrio() < GetState(_start).h || GetState(_start).h == double.PositiveInfinity))
            {
                ProcessState();
            }
            return ExtractPath();
        }
        private double GetH(Node n) => GetState(n).h;

        private (double h, Tag tag, Node parent) GetState(Node n) =>
            _state.TryGetValue(n, out var s) ? s : (double.PositiveInfinity, Tag.NEW, null);

        private void SetState(Node n, double h, Tag tag, Node parent) =>
            _state[n] = (h, tag, parent);

        private void Insert1(Node n, double h) =>
            _open.Enqueue(n, h);
        public void UpdateCost(Edge edge, double newCost)
        {
            // 边变化影响 From 和 To 的 h 值
            ModifyCost(edge, newCost);
            ComputePath();
        }

        /* ---------- 内部 ---------- */
        private IEnumerable<Node> AllNodes() =>
            // 找到所有出现的节点
            new HashSet<Node>(
                _start.OutEdges.Select(e => e.To)
                .Concat(_start.InEdges.Select(e => e.From))
                .Concat(_goal.OutEdges.Select(e => e.To))
                .Concat(_goal.InEdges.Select(e => e.From))
                .Append(_start).Append(_goal));

        private IEnumerable<Node> Succ(Node n) => n.InEdges.Select(e => e.From);
        private IEnumerable<Node> Pred(Node n) => n.OutEdges.Select(e => e.To);

        private double EdgeCost(Node from, Node to) =>
            from.OutEdges.FirstOrDefault(e => e.To == to)?.Cost ?? double.PositiveInfinity;

        private void Insert(Node n, double hNew)
        {
            _open.Enqueue(n, hNew);
        }

        private void ProcessState()
        {
            var n = _open.Dequeue();
            var kOld = _open.Count > 0 ? _open.PeekPrio() : double.PositiveInfinity;
            var hOld = _state[n].h;

            if (kOld < hOld)                  // RAISE
            {
                foreach (var p in Pred(n))
                {
                    if (_state[p].h <= kOld && hOld > _state[p].h + EdgeCost(n, p))
                    {
                        _state[n] = (_state[p].h + EdgeCost(n, p), _state[n].tag, p);
                        Insert(n, _state[n].h);
                    }
                }
            }
            else                              // LOWER
            {
                SetState(n, _state[n].h, Tag.CLOSED, _state[n].parent);
                foreach (var s in Succ(n))
                {
                    var newH = _state[n].h + EdgeCost(s, n);
                    var (sh, st, sp) = GetState(s);
                    if (st == Tag.NEW ||
                        (sp == n && sh != newH) ||
                        (sp != n && sh > newH))
                    {
                        SetState(s, newH, Tag.OPEN, n);
                        Insert(s, newH);
                    }
                }
            }
        }

        private void ModifyCost(Edge edge, double newValue)
        {
            var n = edge.From;
            var s = edge.To;
            
            if (_state[n].tag == Tag.CLOSED)
            {
                if (newValue == double.PositiveInfinity)
                {
                    Insert(n, double.PositiveInfinity);
                }
                else
                {
                    Insert(n, _state[s].h + newValue);
                }
            }
                
            //while (_open.Count > 0 && _open.PeekPrio() < _state[_start].h)
            //    ProcessState();
        }

        private List<Node> ExtractPath()
        {
            if (_state[_start].h == double.PositiveInfinity) return null;

            var path = new List<Node>();
            for (var cur = _start; cur != null; cur = _state[cur].parent)
            {
                path.Add(cur);
                if (cur == _goal) break;
            }
            return path;
        }

        public static void Test()
        {
            // 建一条链 0 → 1 → 2 → 3
            var n0 = new Node(0);
            var n1 = new Node(1);
            var n2 = new Node(2);
            var n3 = new Node(3);

            var e01 = new Edge(n0, n1, 1);
            var e12 = new Edge(n1, n2, 2);
            var e23 = new Edge(n2, n3, 1);
            var e03 = new Edge(n0, n3, 100);
            n0.OutEdges.Add(e03); n3.InEdges.Add(e03);
            n0.OutEdges.Add(e01); n1.InEdges.Add(e01);
            n1.OutEdges.Add(e12); n2.InEdges.Add(e12);
            n2.OutEdges.Add(e23); n3.InEdges.Add(e23);
            var loop0 = new Edge(n0, n0, 0);
            n0.InEdges.Add(loop0); n0.OutEdges.Add(loop0);
            var ds = new DStarDirected(n3);
            ds.SetStartNode(n0);

            Console.WriteLine("首次路径: " +
                string.Join("→", ds.ComputePath()?.Select(n => n.Id.ToString()) ?? new[] { "无通路" }));

            // 动态把 1→2 的权值 2 → 100
            ds.UpdateCost(e01, double.PositiveInfinity);
            Console.WriteLine("改权后路径: " +
                string.Join("→", ds.ComputePath()?.Select(n => n.Id.ToString()) ?? new[] { "无通路" }));
            ds.SetStartNode(n1);
            Console.WriteLine("改权后路径: " +
                string.Join("→", ds.ComputePath()?.Select(n => n.Id.ToString()) ?? new[] { "无通路" }));
            Console.WriteLine("按 <Enter> 退出");
            Console.ReadLine();
        }
    }

}
