using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Slb.Ocean.Basics;
using Slb.Ocean.Core;
using Slb.Ocean.Geometry;
using Slb.Ocean.Petrel.DomainObject;
using Slb.Ocean.Petrel.DomainObject.PillarGrid;

namespace DigitalFrac.Model
{
    public class FracConnection
    {
        public Point2[] Boundary { get; set; }
        public Index3 Cell { get; set; }
        public double Area { get; set; }
        public double Height { get; set; }
        public double BlockPermeability { get; set; }
        public double SquareDrainage { get; set; }
        public double CircularDrainage { get; set; }
        public double BlockPressureRadius { get; set; }
        public double WingLength { get; set; }
        public double PenetrationRatio { get; set; }
        public double FractureConductivity { get; set; }
        public double ProppantNumber { get; set; }
        public double EquivalentRadius { get; set; }
        public double PseudoSkin { get; set; }
        public double GridBlockTerm { get; set; }
        public double FracPlaneTerm { get; set; }
        public double Transmissibility { get; set; }

        public class FacetFactory
        {
            private FracFacet _fracFacet;
            private IVoxel _voxel;
            private IPermeable _perm;
            private IActive _active;
            private Matrix4 _scene;
            private SquareDrainageZ _squareDrainage;
            private CircularDrainageZ _circularDrainage;
            private BlockPressureEquivalentZ _blockPressureEquivalent;

            public FacetFactory(FracFacet fracFacet, IVoxel voxel, IPermeable perm, IActive active)
            {
                _fracFacet = fracFacet;
                _voxel = voxel;
                _perm = perm;
                _active = active;

                _scene = new Matrix4();
                Facet facet = fracFacet.Shape;
                _scene.Translate(-facet.Plane.DefiningPoint.X, -facet.Plane.DefiningPoint.Y, -facet.Plane.DefiningPoint.Z);
                Vector3 Oy = new Vector3(0.0, 1.0, 0.0);
                double a = Math.Acos(Vector3.Dot(Oy, facet.Plane.Normal.NormalizedVector));
                _scene.Rotate(0.0, 0.0, -a, false);

                _squareDrainage = new SquareDrainageZ(voxel);
                _circularDrainage = new CircularDrainageZ(voxel);
                _blockPressureEquivalent = new BlockPressureEquivalentZ(voxel, perm);
            }

            public bool IsValid(FacetCellIntersection fci)
            {
                return fci.Points.Length > 2 && _active.IsActive(fci.CellIndex);
            }

            public FracConnection Make(FacetCellIntersection fci)
            {
                Point3[] pos3 = (Point3[])fci.Points.Clone();
                _scene.TransformPoints(pos3);
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
                double dz = _voxel.Dz(ci);
                double xf = 0.5 * area / dz;
                double xe = 2.0 * _squareDrainage.Radius(ci);
                double blockRadius = _blockPressureEquivalent.Radius(ci);
                double Ix = 2.0 * xf / xe;
                double k = _perm.Kz_GeometricMean(ci);
                double Cfd = _fracFacet.FracPerm * _fracFacet.Aperture / (k * xf);
                double Nprop = Ix * Ix * Cfd;
                double u = Math.Log(Cfd);
                double f = ((0.116 * u - 0.328) * u + 1.65) / (((0.005 * u + 0.064) * u + 0.18) * u + 1.0);
                double r = xf * Math.Exp(-f);
                double s = Math.Log(0.5 * _fracFacet.BoreholeDiameter / r);
                double blockT = 2.0 * Math.PI * k * dz / (Math.Log(xe / _fracFacet.BoreholeDiameter) + s);

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
                    Vector2 A = n.ToNormalized() * tangent.Norm * _fracFacet.Aperture * _fracFacet.FracPerm;

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
                    BlockPermeability = k,
                    WingLength = xf,
                    SquareDrainage = 0.5 * xe,
                    CircularDrainage = _circularDrainage.Radius(ci),
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

        }
    }
}
