using System;
using System.Collections.Generic;
using System.Drawing;
using System.Net.Http.Json;
using System.Text.Json.Serialization;
using static System.Net.Mime.MediaTypeNames;

namespace DStar2D
{
    public class DStar
    {
        private readonly Point _sStart;
        private readonly Point _sGoal;
        private readonly Environment _env;

        private readonly HashSet<Point> _open = new();
        private readonly Dictionary<Point, string> _t = new();
        private readonly Dictionary<Point, Point> _parent = new();
        private readonly Dictionary<Point, double> _h = new();
        private readonly Dictionary<Point, double> _k = new();
        private List<Point> _path = new();
        private HashSet<Point> _Obs = new();

        private HashSet<Point> _visited = new();
        private int _count = 0;

        public DStar(Point sStart, Point sGoal, HashSet<Point> obs)
        {
            _sStart = sStart;
            _sGoal = sGoal;
            _env = new Environment();
            _Obs = obs;
        }

        public void Init()
        {
            for (int i = 0; i < _env.XRange; i++)
            {
                for (int j = 0; j < _env.YRange; j++)
                {
                    _t[new Point(i, j)] = "NEW";
                    _k[new Point(i, j)] = 0.0;
                    _h[new Point(i, j)] = double.PositiveInfinity;
                    _parent[new Point(i, j)] = new(-1,-1);
                }
            }

            _h[_sGoal] = 0.0;
        }

        public void Run()
        {
            Init();
            Insert(_sGoal, 0);

            while (true)
            {
                ProcessState();
                if (_t[_sStart] == "CLOSED")
                {
                    break;
                }
            }

            _path = ExtractPath(_sStart, _sGoal);
        }

        public void SetObs(Point point, bool isObs)
        {
            if (isObs)
            {
                if (!_Obs.Contains(point))
                {
                    _Obs.Add(point);
                }
            }
            else if (_Obs.Contains(point))
            {
                _Obs.Remove(point);
            }
        }

        private List<Point> ExtractPath(Point sStart, Point sEnd)
        {
            var path = new List<Point> { sStart };
            var s = sStart;
            while (true)
            {
                s = _parent[s];
                path.Add(s);
                if (s == sEnd)
                {
                    Console.WriteLine($"{string.Join("→", path.Select(n => "(" + n.X.ToString() + ", " + n.Y.ToString() + ")") ?? new[] { "无通路" })}");
                    return path;
                }
            }
        }

        private double ProcessState()
        {
            var s = MinState();
            _visited.Add(s);

            if (s.X == -1 && s.Y == -1)
            {
                return double.PositiveInfinity;
            }

            var kOld = GetKMin();
            Delete(s);

            // RAISE state (increased cost)
            if (kOld < _h[s])
            {
                foreach (var sN in GetNeighbor(s))
                {
                    if (_h[sN] <= kOld && _h[s] > _h[sN] + Cost(sN, s))
                    {
                        _parent[s] = sN;
                        _h[s] = _h[sN] + Cost(sN, s);
                    }
                }
            }

            // LOWER state (cost reductions)
            if (kOld == _h[s])
            {
                foreach (var sN in GetNeighbor(s))
                {
                    if (_t[sN] == "NEW" ||
                        (_parent[sN] == s && _h[sN] != _h[s] + Cost(s, sN)) ||
                        (_parent[sN] != s && _h[sN] > _h[s] + Cost(s, sN)))
                    {
                        _parent[sN] = s;
                        Insert(sN, _h[s] + Cost(s, sN));
                    }
                }
            }
            else
            {
                foreach (var sN in GetNeighbor(s))
                {
                    if (_t[sN] == "NEW" ||
                        (_parent[sN] == s && _h[sN] != _h[s] + Cost(s, sN)))
                    {
                        _parent[sN] = s;
                        Insert(sN, _h[s] + Cost(s, sN));
                    }
                    else
                    {
                        if (_parent[sN] != s && _h[sN] > _h[s] + Cost(s, sN))
                        {
                            Insert(s, _h[s]);
                        }
                        else
                        {
                            if (_parent[sN] != s && _h[s] > _h[sN] + Cost(sN, s) && _t[sN] == "CLOSED" && _h[sN] > kOld)
                            {
                                Insert(sN, _h[sN]);
                            }
                        }
                    }
                }
            }
            return GetKMin();
        }

        private Point MinState()
        {
            if (_open.Count == 0)
            {
                return new(-1,-1);
            }

            return _open.MinBy(p => _k[p]);
        }

        private double GetKMin()
        {
            if (_open.Count == 0)
            {
                return -1;
            }

            return _open.Min(p => _k[p]);
        }

        private void Insert(Point s, double hNew)
        {
            if (_t[s] == "NEW")
            {
                _k[s] = hNew;
            }
            else if (_t[s] == "OPEN")
            {
                _k[s] = Math.Min(_k[s], hNew);
            }
            else if (_t[s] == "CLOSED")
            {
                _k[s] = Math.Min(_h[s], hNew);
            }

            _h[s] = hNew;
            _t[s] = "OPEN";
            _open.Add(s);
        }

        private void Delete(Point s)
        {
            if (_t[s] == "OPEN")
            {
                _t[s] = "CLOSED";
            }

            _open.Remove(s);
        }

        public void Modify(Point s)
        {
            ModifyCost(s);

            while (true)
            {
                var kMin = ProcessState();

                if (kMin >= _h[s])
                {
                    break;
                }
            }
            ExtractPath(_sStart, _sGoal);
        }

        private void ModifyCost(Point s)
        {
            if (_t[s] == "CLOSED")
            {
                Insert(s, _h[_parent[s]] + Cost(s, _parent[s]));
            }
        }

        private IEnumerable<Point> GetNeighbor(Point s)
        {
            var neighbors = new List<Point>();

            foreach (var u in _env.Motions)
            {
                int newX = s.X + u.X, newY = s.Y + u.Y;
                if (newX < 0 || newX == _env.XRange || newY < 0 || newY == _env.YRange) continue;
                var sNext = new Point(s.X + u.X, s.Y + u.Y);
                if (!_Obs.Contains(sNext))
                {
                    neighbors.Add(sNext);
                }
            }

            return neighbors;
        }

        private double Cost(Point sStart, Point sGoal)
        {
            if (IsCollision(sStart, sGoal))
            {
                return double.PositiveInfinity;
            }
            return _env.GetEdgeCost(sStart, sGoal);
        }

        private bool IsCollision(Point sStart, Point sGoal)
        {
            if (_Obs.Contains(sStart) || _Obs.Contains(sGoal))
            {
                return true;
            }

            if (sStart.X != sGoal.X && sStart.Y != sGoal.Y)
            {
                if (sGoal.X - sStart.X == sStart.Y - sGoal.Y)
                {
                    var s1 = new Point(Math.Min(sStart.X, sGoal.X), Math.Min(sStart.Y, sGoal.Y));
                    var s2 = new Point(Math.Max(sStart.X, sGoal.X), Math.Max(sStart.Y, sGoal.Y));
                    if (_Obs.Contains(s1) || _Obs.Contains(s2))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        private void PlotPath(List<Point> path)
        {
            var px = path.Select(p => p.X).ToArray();
            var py = path.Select(p => p.Y).ToArray();
        }

        public void UpdateEdgeCost(Point from, Point to, double newCost)
        {
            _env.UpdateEdgeCost(from, to, newCost);
            Modify(from);
            //_path = ExtractPath(_sStart, _sGoal);
            //Console.WriteLine($"{string.Join("→", _path.Select(n => "("+n.X.ToString() +", " + n.Y.ToString() +")") ?? new[] { "无通路" })}");
        }
    }

    public class Environment
    {
        public int XRange { get; } = 50;
        public int YRange { get; } = 30;
        public Dictionary<Tuple<Point, Point>, double> EdgeCosts { get; } = new();

        public Point[] Motions { get; } = new Point[]
        {
            new Point(-1, 0), new Point(0, -1), new Point(1, 0), new Point(0, 1),
            new Point(-1, -1), new Point(-1, 1), new Point(1, -1), new Point(1, 1)
        };

        public void UpdateEdgeCost(Point from, Point to, double newCost)
        {
            var key = Tuple.Create(from, to);
            EdgeCosts[key] = newCost;
        }

        public double GetEdgeCost(Point from, Point to)
        {
            var key = Tuple.Create(from, to);
            return EdgeCosts.ContainsKey(key) ? EdgeCosts[key] : Math.Sqrt(Math.Pow(to.X - from.X, 2) + Math.Pow(to.Y - from.Y, 2));
        }
    }

    internal static class Program
    {
        private static void Main()
        {
            var sStart = new Point(5, 5);
            var sGoal = new Point(45, 25);
            var dstar = new DStar(sStart, sGoal, []);
            dstar.Run();

            // Example of updating edge cost
            dstar.UpdateEdgeCost(new Point(10, 10), new Point(11, 11), 15.0);
            //dstar.SetObs(new Point(10, 10), true);
            //dstar.Modify(new Point(10, 10));
        }
    }
}