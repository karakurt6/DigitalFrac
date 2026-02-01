using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Slb.Ocean.Basics;
using Slb.Ocean.Core;
using Slb.Ocean.Geometry;
using Slb.Ocean.Petrel;
using Slb.Ocean.Petrel.DomainObject;
using Slb.Ocean.Petrel.DomainObject.PillarGrid;

namespace DigitalFrac.Model
{
    public class RefinementProcessing : IVoxel, IPermeable, IActive
    {
        private LocalGrid _grid;
        private LocalProperty _kx;
        private LocalProperty _ky;
        private LocalProperty _kz;
        private LocalProperty _ntg;

        public RefinementProcessing(LocalGrid grid, string kx, string ky, string kz, string ntg)
        {
            _grid = grid;
            _kx = grid.GetProperty(grid.LocalGridSet.Grid.Properties.FirstOrDefault(item => item.Name.Equals(kx, StringComparison.OrdinalIgnoreCase)));
            _ky = grid.GetProperty(grid.LocalGridSet.Grid.Properties.FirstOrDefault(item => item.Name.Equals(ky, StringComparison.OrdinalIgnoreCase)));
            _kz = grid.GetProperty(grid.LocalGridSet.Grid.Properties.FirstOrDefault(item => item.Name.Equals(kz, StringComparison.OrdinalIgnoreCase)));
            _ntg = _grid.GetProperty(_grid.LocalGridSet.Grid.Properties.FirstOrDefault(item => item.Name.Equals(ntg, StringComparison.OrdinalIgnoreCase)));
        }

        static private double[] BilinearInterpolationCoefficients(double u, double v)
        {
            return new double[4] { (1.0 - u) * (1.0 - v), u * (1.0 - v), (1.0 - u) * v, u * v };
        }

        private double Dz_BilinearInterpolation(Index3 cell)
        {
            double[] coef = BilinearInterpolationCoefficients(0.5, 0.5);

            CellCorner[] topCorners = { CellCorner.TopSouthWest, CellCorner.TopSouthEast, CellCorner.TopNorthWest, CellCorner.TopNorthEast };
            Vector3 topCenter = _grid.GetCellCorners(cell, topCorners).Select(p => new Vector3(p.X, p.Y, p.Z)).Zip(coef, (x, y) => x * y).Aggregate((x, y) => x + y);
            // Vector3 topCenter = blerp(topNodes, 0.5, 0.5);

            CellCorner[] baseCorners = { CellCorner.BaseSouthWest, CellCorner.BaseSouthEast, CellCorner.BaseNorthWest, CellCorner.BaseNorthEast };
            Vector3 baseCenter = _grid.GetCellCorners(cell, baseCorners).Select(p => new Vector3(p.X, p.Y, p.Z)).Zip(coef, (x, y) => x * y).Aggregate((x, y) => x + y);
            // Vector3 baseCenter = blerp(topNodes, 0.5, 0.5);

            return Vector3.Subtract(topCenter, baseCenter).Norm;
        }
        private double Dx_BilinearInterpolation(Index3 cell)
        {
            double[] coef = BilinearInterpolationCoefficients(0.5, 0.5);

            CellCorner[] westCorners = { CellCorner.TopSouthWest, CellCorner.TopNorthWest, CellCorner.BaseSouthWest, CellCorner.BaseNorthWest };
            Vector3 westCenter = _grid.GetCellCorners(cell, westCorners).Select(p => new Vector3(p.X, p.Y, p.Z)).Zip(coef, (x, y) => x * y).Aggregate((x, y) => x + y);
            // Vector3 westCenter = blerp(westNodes, 0.5, 0.5);

            CellCorner[] eastCorners = { CellCorner.TopSouthEast, CellCorner.TopNorthEast, CellCorner.BaseSouthEast, CellCorner.BaseNorthEast };
            Vector3 eastCenter = _grid.GetCellCorners(cell, eastCorners).Select(p => new Vector3(p.X, p.Y, p.Z)).Zip(coef, (x, y) => x * y).Aggregate((x, y) => x + y);
            // Vector3 eastCenter = blerp(eastNodes, 0.5, 0.5);

            return Vector3.Subtract(westCenter, eastCenter).Norm;
        }
        private double Dy_BilinearInterpolation(Index3 cell)
        {
            double[] coef = BilinearInterpolationCoefficients(0.5, 0.5);

            CellCorner[] southCorners = { CellCorner.TopSouthWest, CellCorner.TopSouthEast, CellCorner.BaseSouthWest, CellCorner.BaseSouthEast };
            Vector3 southCenter = _grid.GetCellCorners(cell, southCorners).Select(p => new Vector3(p.X, p.Y, p.Z)).Zip(coef, (x, y) => x * y).Aggregate((x, y) => x + y);
            // Vector3 southCenter = blerp(southNodes, 0.5, 0.5);

            CellCorner[] northCorners = { CellCorner.TopNorthWest, CellCorner.TopNorthEast, CellCorner.BaseNorthWest, CellCorner.BaseNorthEast };
            Vector3 northCenter = _grid.GetCellCorners(cell, northCorners).Select(p => new Vector3(p.X, p.Y, p.Z)).Zip(coef, (x, y) => x * y).Aggregate((x, y) => x + y);
            // Vector3 northCenter = blerp(northNodes, 0.5, 0.5);

            return Vector3.Subtract(southCenter, northCenter).Norm;
        }

        public double Dx(Index3 cell)
        {
            return Dx_BilinearInterpolation(cell);
        }

        public double Dy(Index3 cell)
        {
            return Dy_BilinearInterpolation(cell);
        }

        public double Dz(Index3 cell)
        {
            return Dz_BilinearInterpolation(cell);
        }

        public double Kx(Index3 cell)
        {
            return PetrelUnitSystem.ConvertToUI(_kx.Template, _kx[cell]);
        }

        public double Ky(Index3 cell)
        {
            return PetrelUnitSystem.ConvertToUI(_ky.Template, _ky[cell]);
        }

        public double Kz(Index3 cell)
        {
            return PetrelUnitSystem.ConvertToUI(_kz.Template, _kz[cell]);
        }

        public double Kz_GeometricMean(Index3 cell)
        {
            return Math.Sqrt(Kx(cell) * Ky(cell));
        }

        public double Kx_GeometricMean(Index3 cell)
        {
            return Math.Sqrt(Kz(cell) * Ky(cell));
        }

        public double Ky_GeometricMean(Index3 cell)
        {
            return Math.Sqrt(Kx(cell) * Kz(cell));
        }

        public bool IsActive(Index3 cell)
        {
            return _ntg[cell] > 0.0;
        }

        public IEnumerable<FacetCellIntersection> GetIntersection(Facet facet)
        {
            ILocalGridIntersectionService localGridIntersectionService = CoreSystem.GetService<ILocalGridIntersectionService>();
            return localGridIntersectionService.GetLocalGridPlaneIntersection(_grid, facet);
        }

        /*
        public IEnumerable<FracConnection> Connect(string positiveProp, FracFacet fracFacet)
        {
            Facet facet = fracFacet.Shape;
            ILocalGridIntersectionService localGridIntersectionService = CoreSystem.GetService<ILocalGridIntersectionService>();
            IEnumerable<FacetCellIntersection> cells = localGridIntersectionService.GetLocalGridPlaneIntersection(_grid, facet);
            LocalProperty ntg = _grid.GetProperty(_grid.LocalGridSet.Grid.Properties.FirstOrDefault(item => item.Name.Equals(positiveProp, StringComparison.OrdinalIgnoreCase)));

            //Index3 numCells = _grid.NumCellsIJK;
            //int IndexToNaturalOrder(Index3 index) => index.I + numCells.I * (index.J + numCells.J * index.K);
            //Index3 NaturalOrderToIndex(int num) => new Index3(num % numCells.I, (num % (numCells.I * numCells.J)) / numCells.I, num / (numCells.I * numCells.J));

            // affine transform from 3D world coordinates system to 2D local coordinate system
            Matrix4 T = new Matrix4();
            T.Translate(-facet.Plane.DefiningPoint.X, -facet.Plane.DefiningPoint.Y, -facet.Plane.DefiningPoint.Z);
            Vector3 Oy = new Vector3(0.0, 1.0, 0.0);
            double a = Math.Acos(Vector3.Dot(Oy, facet.Plane.Normal.NormalizedVector));
            T.Rotate(0.0, 0.0, -a, false);

            SquareDrainageZ squareDrainage = new SquareDrainageZ(this);
            CircularDrainageZ circularDrainage = new CircularDrainageZ(this);
            BlockPressureEquivalentZ blockDrainage = new BlockPressureEquivalentZ(this);

            FracConnection Make(FacetCellIntersection fci)
            {
                Point3[] pos3 = (Point3[])fci.Points.Clone();
                T.TransformPoints(pos3);
                Point2[] pos2 = pos3.AsEnumerable().Select(item => new Point2(item.X, item.Z)).ToArray();

                Point2 prev = pos2.Last();
                double area = 0.0;
                foreach (Point2 curr in pos2)
                {
                    area += curr.X * prev.Y - prev.X * curr.Y;
                    prev = curr;
                }
                area *= 0.5;
                Point2 centerPos = new Point2(pos2.Average(item => item.X), pos2.Average(item => item.Y));

                Index3 ci = fci.CellIndex;
                double dz = Dz(ci);
                double xf = 0.5 * area / dz;
                double xe = 2.0 * squareDrainage.Radius(ci);
                double blockRadius = blockDrainage.Radius(ci);
                double Ix = 2.0 * xf / xe;
                double k = fracFacet.MultPerm * KmeanZ(ci);
                double Cfd = fracFacet.FracPerm * fracFacet.Aperture / (k * xf);
                double Nprop = Ix * Ix * Cfd;
                double u = Math.Log(Cfd);
                double f = ((0.116 * u - 0.328) * u + 1.65) / (((0.005 * u + 0.064) * u + 0.18) * u + 1.0);
                double r = xf * Math.Exp(-f);
                double s = Math.Log(0.5 * fracFacet.BoreholeDiameter / r);
                double blockT = 2.0 * Math.PI * k * dz / (Math.Log(xe / fracFacet.BoreholeDiameter) + s);

                // compute transmissibility term inside fracture plane
                double fracT = 0.0;
                prev = pos2.Last();
                foreach (Point2 curr in pos2)
                {
                    Vector2 tangent = curr - prev;
                    Point2 origin = prev + 0.5 * tangent;
                    Vector2 q = new Vector2(-origin.X, -origin.Y);
                    Vector2 d = tangent.ToNormalized();
                    Vector2 n = d * Vector2.Dot(d, q) - q;
                    Vector2 A = n.ToNormalized() * tangent.Norm * fracFacet.Aperture * fracFacet.FracPerm;

                    // check if vector from facet center and A is situated on one halfplane
                    double Aq = Vector2.Dot(A, q);
                    if (Aq < 0.0)
                    {
                        A = Vector2.Negate(A);
                    }

                    // check if plane is exposed to flow from source
                    q = origin - centerPos;
                    Aq = Vector2.Dot(A, q);
                    if (Aq > 0.0)
                    {
                        fracT += Aq / Vector2.Dot(q, q);
                    }
                    prev = curr;
                }

                const double c = 0.008527;
                double t = c * blockT;
                if (fracT > 0.0)
                {
                    t = c / (1.0 / blockT + 1.0 / fracT);
                }

                return new FracConnection()
                {
                    Boundary = pos2,
                    Cell = ci,
                    Area = area,
                    Height = dz,
                    WingLength = xf,
                    SquareDrainage = 0.5 * xe,
                    CircularDrainage = circularDrainage.Radius(ci),
                    BlockPressureRadius = blockRadius,
                    PenetrationRatio = Ix,
                    FractureConductivity = Cfd,
                    ProppantNumber = Nprop,
                    EquivalentRadius = r,
                    PseudoSkin = s,
                    GridBlockTerm = blockT,
                    FracPlaneTerm = fracT,
                    Transmissibility = t
                };
            }

            return cells.Where(item => item.Points.Length > 2 && ntg[item.CellIndex] > 0.0).Select(item => Make(item));
        }
        */
    }
}
