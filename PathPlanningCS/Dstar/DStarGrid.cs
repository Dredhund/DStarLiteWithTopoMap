using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathPlanningCS.Dstar
{
    #region ==== = 基础类型 ==== =
    public readonly record struct Pt(int X, int Y)
    {
        public static Pt operator +(Pt a, Pt b) => new(a.X + b.X, a.Y + b.Y);
        public double Dist(Pt other) => Math.Sqrt((X - other.X) * (X - other.X) +
            (Y - other.Y) * (Y - other.Y));
    }
    #endregion

    #region ==== = 网格环境 ==== =
    public class Env
    {
        public int W { get; }
        public int H { get; }
        public HashSet<Pt> Obs { get; } = new();   // 障碍坐标
        public static readonly Pt[] Dirs8 =
        {
        new(-1,-1), new(-1,0), new(-1,1),
        new(0,-1),            new(0,1),
        new(1,-1), new(1,0), new(1,1)
    };

        public Env(int w, int h) { W = w; H = h; }

        public bool InRange(Pt p) => p.X >= 0 && p.X < W && p.Y >= 0 && p.Y < H;
        public bool IsObs(Pt p) => Obs.Contains(p);
        public bool IsFree(Pt p) => InRange(p) && !IsObs(p);
    }
    #endregion

    public sealed class DStarGrid
    {
        private readonly Env _env;
        private readonly Dictionary<Pt, (double h, Tag tag, Pt parent)> _state = new();
        private readonly PrioQueue<Pt, double> _open = new();
        private Pt _start, _goal;

        public DStarGrid(Env env, Pt start, Pt goal)
        {
            _env = env; _start = start; _goal = goal;
        }

        /* ------------ 公开接口 ------------ */
        public void Init()
        {
            _state.Clear(); _open.Clear();
            foreach (var p in AllPts())
                _state[p] = (double.PositiveInfinity, Tag.NEW, default);
            _state[_goal] = (0, Tag.NEW, default);
            Insert(_goal, 0);
        }

        public List<Pt> ComputePath()
        {
            while (_open.Count > 0 &&
                (_open.PeekPrio() < _state[_start].h || _state[_start].h == double.PositiveInfinity))
            {
                ProcessState();
            }
            return ExtractPath();
        }

        public void AddObstacle(Pt p)
        {
            if (!_env.IsFree(p)) return;
            _env.Obs.Add(p);

            // 把与 p 相邻的 8 个方向都触发 cost 变化
            foreach (var d in Env.Dirs8)
            {
                var n = p + d;
                if (!_env.InRange(n)) continue;
                ModifyCost(n);
            }
            ComputePath();
        }

        public void RemoveObstacle(Pt p)
        {
            if (!_env.Obs.Contains(p)) return;
            _env.Obs.Remove(p);

            // 通知 8 个邻居 cost 已恢复
            foreach (var d in Env.Dirs8)
            {
                var n = p + d;
                if (!_env.InRange(n)) continue;
                ModifyCost(n);          // 与 AddObstacle 对称
            }
            ComputePath();              // 触发一次增量修正
        }

        /* ------------ 内部实现 ------------ */
        private IEnumerable<Pt> AllPts() =>
            from x in Enumerable.Range(0, _env.W)
            from y in Enumerable.Range(0, _env.H)
            select new Pt(x, y);

        private IEnumerable<Pt> Neighbors(Pt p) =>
            Env.Dirs8.Select(d => p + d).Where(_env.IsFree);

        private double Cost(Pt a, Pt b)
        {
            if (!_env.IsFree(a) || !_env.IsFree(b)) return double.PositiveInfinity;
            return a.Dist(b);
        }

        private void Insert(Pt p, double hNew)
        {
            _state[p] = (hNew, Tag.OPEN, _state[p].parent);
            _open.Enqueue(p, hNew);
        }

        private void ProcessState()
        {
            var p = _open.Dequeue();
            var kOld = _open.Count > 0 ? _open.PeekPrio() : double.PositiveInfinity;
            var hOld = _state[p].h;

            // RAISE / LOWER 分支
            if (kOld < hOld)            // RAISE
            {
                foreach (var n in Neighbors(p))
                {
                    if (_state[n].h <= kOld && hOld > _state[n].h + Cost(n, p))
                    {
                        _state[p] = (_state[n].h + Cost(n, p), _state[p].tag, n);
                        _state[p] = (_state[p].h, Tag.OPEN, n);
                        Insert(p, _state[p].h);
                    }
                }
            }
            else                        // LOWER
            {
                _state[p] = (_state[p].h, Tag.CLOSED, _state[p].parent);
                foreach (var n in Neighbors(p))
                {
                    var newH = _state[p].h + Cost(p, n);
                    if (_state[n].tag == Tag.NEW ||
                        (_state[n].parent == p && _state[n].h != newH) ||
                        (_state[n].parent != p && _state[n].h > newH))
                    {
                        _state[n] = (newH, Tag.OPEN, p);
                        Insert(n, newH);
                    }
                }
            }
        }

        private void ModifyCost(Pt p)
        {
            if (_state[p].tag == Tag.CLOSED)
            {
                Insert(p, _state[p].h);
            }
            while (_open.Count > 0 && _open.PeekPrio() < _state[_start].h)
                ProcessState();
        }

        private List<Pt> ExtractPath()
        {
            if (_state[_start].h == double.PositiveInfinity) return null;
            var path = new List<Pt> { _start };
            var cur = _start;
            while (cur != _goal)
            {
                cur = Neighbors(cur)
                    .OrderBy(n => _state[n].h + Cost(cur, n))
                    .First();
                path.Add(cur);
            }
            return path;
        }

        /* ------------ 控制台可视化 ------------ */
        public void PrintPath(List<Pt> path)
        {
            Console.Clear();
            for (int y = 0; y < _env.H; y++)
            {
                for (int x = 0; x < _env.W; x++)
                {
                    var p = new Pt(x, y);
                    char c = _env.IsObs(p) ? '#' :
                        path != null && path.Contains(p) ? '*' :
                        '.';
                    Console.Write(c);
                }
                Console.WriteLine();
            }
            Console.WriteLine();
        }

        public static void Test()
        {
            const int W = 20, H = 10;
            var env = new Env(W, H);
            var start = new Pt(2, 2);
            var goal = new Pt(17, 7);

            var ds = new DStarGrid(env, start, goal);
            ds.Init();
            var path = ds.ComputePath();
            ds.PrintPath(path);

            // 模拟动态障碍：在 (5,2) (6,2) 放置石块
            ds.AddObstacle(new Pt(5, 2));
            ds.AddObstacle(new Pt(6, 2));
            path = ds.ComputePath();
            ds.PrintPath(path);

            ds.RemoveObstacle(new Pt(5, 2));
            path = ds.ComputePath();
            ds.PrintPath(path);

            Console.WriteLine("按 <Enter> 退出");
            Console.ReadLine();
        }
    }
}
