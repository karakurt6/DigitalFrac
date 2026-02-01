using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Slb.Ocean.Basics;

namespace DigitalFrac.Model
{
    public interface IDrainageArea
    {
        double Radius(Index3 cell);
    }

    public interface IVoxel
    {
        double Dx(Index3 cell);
        double Dy(Index3 cell);
        double Dz(Index3 cell);
    }

    public interface IPermeable
    {
        double Kx(Index3 cell);
        double Ky(Index3 cell);
        double Kz(Index3 cell);
        double Kx_GeometricMean(Index3 cell);
        double Ky_GeometricMean(Index3 cell);
        double Kz_GeometricMean(Index3 cell);
    }

    public interface IActive
    {
        bool IsActive(Index3 cell);
    }

    public class ConstDrainage : IDrainageArea
    {
        double _radius;

        public ConstDrainage(double radius)
        {
            _radius = radius;
        }

        public double Radius(Index3 cell)
        {
            return _radius;
        }
    }

    public class SquareDrainageZ : IDrainageArea
    {
        IVoxel _voxel;

        public SquareDrainageZ(IVoxel voxel)
        {
            _voxel = voxel;
        }

        public double Radius(Index3 cell)
        {
            return 0.5 * Math.Sqrt(_voxel.Dx(cell) * _voxel.Dy(cell));
        }
    }
    public class SquareDrainageX : IDrainageArea
    {
        IVoxel _voxel;

        public SquareDrainageX(IVoxel voxel)
        {
            _voxel = voxel;
        }

        public double Radius(Index3 cell)
        {
            return 0.5 * Math.Sqrt(_voxel.Dz(cell) * _voxel.Dy(cell));
        }
    }

    public class CircularDrainageZ : IDrainageArea
    {
        IVoxel _voxel;

        public CircularDrainageZ(IVoxel voxel)
        {
            _voxel = voxel;
        }

        public double Radius(Index3 cell)
        {
            return Math.Sqrt(_voxel.Dx(cell) * _voxel.Dy(cell) / Math.PI);
        }
    }

    public class CircularDrainageY : IDrainageArea
    {
        IVoxel _voxel;

        public CircularDrainageY(IVoxel voxel)
        {
            _voxel = voxel;
        }

        public double Radius(Index3 cell)
        {
            return Math.Sqrt(_voxel.Dx(cell) * _voxel.Dz(cell) / Math.PI);
        }
    }

    public class CircularDrainageX : IDrainageArea
    {
        IVoxel _voxel;

        public CircularDrainageX(IVoxel voxel)
        {
            _voxel = voxel;
        }

        public double Radius(Index3 cell)
        {
            return Math.Sqrt(_voxel.Dz(cell) * _voxel.Dy(cell) / Math.PI);
        }
    }

    public class BlockPressureEquivalentZ : IDrainageArea
    {
        IVoxel _voxel;
        IPermeable _perm;

        public BlockPressureEquivalentZ(IVoxel voxel, IPermeable perm)
        {
            _voxel = voxel;
            _perm = perm;
        }

        public double Radius(Index3 cell)
        {
            return 0.28 * Math.Pow(Math.Pow(_voxel.Dx(cell), 2.0) * Math.Pow(_perm.Ky(cell) / _perm.Kx(cell), 0.5)
                + Math.Pow(_voxel.Dy(cell), 2.0) * Math.Pow(_perm.Kx(cell) / _perm.Ky(cell), 0.5), 0.5)
                / (Math.Pow(_perm.Ky(cell) / _perm.Kx(cell), 0.25) + Math.Pow(_perm.Kx(cell) / _perm.Ky(cell), 0.25));
        }
    }

    public class BlockPressureEquivalentX : IDrainageArea
    {
        IVoxel _voxel;
        IPermeable _perm;

        public BlockPressureEquivalentX(IVoxel voxel, IPermeable perm)
        {
            _voxel = voxel;
            _perm = perm;
        }

        public double Radius(Index3 cell)
        {
            return 0.28 * Math.Pow(Math.Pow(_voxel.Dy(cell), 2.0) * Math.Pow(_perm.Kz(cell) / _perm.Ky(cell), 0.5)
                + Math.Pow(_voxel.Dz(cell), 2.0) * Math.Pow(_perm.Ky(cell) / _perm.Kz(cell), 0.5), 0.5)
                / (Math.Pow(_perm.Kz(cell) / _perm.Ky(cell), 0.25) + Math.Pow(_perm.Ky(cell) / _perm.Kz(cell), 0.25));
        }
    }

    public class BlockPressureEquivalentY : IDrainageArea
    {
        IVoxel _voxel;
        IPermeable _perm;

        public BlockPressureEquivalentY(IVoxel voxel, IPermeable perm)
        {
            _voxel = voxel;
            _perm = perm;
        }

        public double Radius(Index3 cell)
        {
            return 0.28 * Math.Pow(Math.Pow(_voxel.Dz(cell), 2.0) * Math.Pow(_perm.Kx(cell) / _perm.Kz(cell), 0.5)
                + Math.Pow(_voxel.Dx(cell), 2.0) * Math.Pow(_perm.Kz(cell) / _perm.Kx(cell), 0.5), 0.5)
                / (Math.Pow(_perm.Kx(cell) / _perm.Kz(cell), 0.25) + Math.Pow(_perm.Kz(cell) / _perm.Kx(cell), 0.25));
        }
    }
}
