using System.Collections.Generic;
using System.Linq;
using System;
using System.Data;
using System.Data.SqlClient;
using System.IO;

using Slb.Ocean.Core;
using Slb.Ocean.Basics;
using Slb.Ocean.Petrel.DomainObject;
using Slb.Ocean.Petrel.DomainObject.Well;
using Slb.Ocean.Geometry;
using Slb.Ocean.Petrel;
using Slb.Ocean.Petrel.DomainObject.PillarGrid;
using Slb.Ocean.Petrel.DomainObject.Intersect.FieldManagement;

namespace DigitalFrac.Model
{
    public class TrajectoryProcessing
    {
        private List<TrajectoryPolylineRecord> trajectory;

        private class MD_Comparer: Comparer<TrajectoryPolylineRecord>
        {
            public override int Compare(TrajectoryPolylineRecord x, TrajectoryPolylineRecord y)
            {
                return x.MD.CompareTo(y.MD);
            }
        }
        
        static public Vector3 Lerp(double t, Vector3 s, Vector3 f)
        {
            return Vector3.Add(s, Vector3.Multiply(t, Vector3.Subtract(f, s)));
        }

        static public Vector3 Nlerp(double t, Vector3 s, Vector3 f)
        {
            Vector3 result = Lerp(t, s, f);
            return Vector3.Divide(result, result.Norm);
        }

        static public Vector3 Slerp(double t, Vector3 s, Vector3 f)
        {
            // dot product -- the cosine of the angle between two vectors
            double dot = Vector3.Dot(s, f);
            // clamp it to range of Acos()
            // this may be unnecessary, but floating point
            // precision can be fickle mistress
            if (dot < -1.0)
            {
                dot = -1.0;
            }
            else if (dot > 1.0)
            {
                dot = 1.0;
            }
            // Acos(dot) returns the angle between start and end,
            // And multiplying that by t returns the angle between
            // start and final result
            double theta = Math.Acos(dot) * t;
            Vector3 relativeVec = Vector3.Subtract(f, Vector3.Multiply(t, s));
            // the final result
            return Vector3.Add(Vector3.Multiply(Math.Cos(theta), s), Vector3.Multiply(Math.Sin(theta) / relativeVec.Norm, relativeVec));
        }

        static public TrajectoryPolylineRecord Interpolate(double t, TrajectoryPolylineRecord start, TrajectoryPolylineRecord final)
        {
            double md = start.MD + t * (final.MD - start.MD);
            Point3 point = start.Point + t * (final.Point - start.Point);
            Vector3 g = Slerp(t, new Vector3(new Angle(start.Inclination), new Angle(start.Azimuth)), new Vector3(new Angle(final.Inclination), new Angle(final.Azimuth)));
            return new TrajectoryPolylineRecord(md, g.Dip.Radians, g.Azimuth.Radians, point);
        }

        public TrajectoryProcessing(IEnumerable<TrajectoryPolylineRecord> input)
        {
            trajectory = new List<TrajectoryPolylineRecord>(input);
        }

        public TrajectoryProcessing()
        {
            trajectory = null;
        }

        public IEnumerable<TrajectoryPolylineRecord> Trajectory()
        {
            return trajectory;
        }

        public void Split(double depth, TrajectoryProcessing upper, TrajectoryProcessing lower)
        {
            if (trajectory != null && trajectory.Count > 0)
            {
                if (trajectory.First().MD >= depth)
                {
                    upper.trajectory = new List<TrajectoryPolylineRecord>();
                    lower.trajectory = new List<TrajectoryPolylineRecord>(trajectory);
                }
                else if (trajectory.Last().MD <= depth)
                {
                    upper.trajectory = new List<TrajectoryPolylineRecord>(trajectory);
                    lower.trajectory = new List<TrajectoryPolylineRecord>();
                }
                else
                {
                    int n = trajectory.BinarySearch(new TrajectoryPolylineRecord(depth, 0.0, 0.0, Point3.Zero), new MD_Comparer());
                    if (n < 0)
                    {
                        int i = ~n;
                        /*
                        if (trajectory[i-1].MD >= depth || trajectory[i].MD <= depth)
                        {
                            PetrelLogger.InfoOutputWindow("Error in BinarySearch() logic detected");
                        }
                        */
                        double t = (depth - trajectory[i-1].MD) / (trajectory[i].MD - trajectory[i-1].MD);
                        TrajectoryPolylineRecord target = Interpolate(t, trajectory[i-1], trajectory[i]);
                        /*
                        if (Math.Abs(target.MD - depth) > double.Epsilon)
                        {
                            PetrelLogger.InfoOutputWindow("Error in Interpolation() routine");
                        }
                        */
                        upper.trajectory = new List<TrajectoryPolylineRecord>(trajectory.GetRange(0, i));
                        lower.trajectory = new List<TrajectoryPolylineRecord>(trajectory.GetRange(i, trajectory.Count - i));
                        upper.trajectory.Add(target);
                        lower.trajectory.Insert(0, target);
                    }
                    else
                    {
                        upper.trajectory = new List<TrajectoryPolylineRecord>(trajectory.GetRange(0, n));
                        lower.trajectory = new List<TrajectoryPolylineRecord>(trajectory.GetRange(n, trajectory.Count - n));
                    }
                }
            }

        }

        public class Comparer: IEqualityComparer<TrajectoryProcessing>
        {
            public bool Equals(TrajectoryProcessing lhs, TrajectoryProcessing rhs)
            {
                if (lhs == null && rhs == null)
                {
                    return true;
                }
                if (lhs == null || rhs == null)
                {
                    return false;
                }
                TrajectoryPolylineRecord lhsRecord = lhs.Trajectory().First();
                TrajectoryPolylineRecord rhsRecord = rhs.Trajectory().First();
                if (lhsRecord == null || rhsRecord == null)
                {
                    return false;
                }
                return Math.Abs(lhsRecord.MD - rhsRecord.MD) < double.Epsilon;
            }

            public int GetHashCode(TrajectoryProcessing traj)
            {
                if (traj == null)
                {
                    return 0;
                }
                IEnumerable<TrajectoryPolylineRecord> rec = traj.Trajectory();
                if (!rec.Any())
                {
                    return 0;
                }
                return rec.First().MD.GetHashCode();
            }
        }
        
        public static Dictionary<TrajectoryProcessing, SegmentCellIntersection> FilterSegmentation(Dictionary<TrajectoryProcessing, SegmentCellIntersection> input, double reservoirStart, double reservoirFinal)
        {
            Dictionary<TrajectoryProcessing, SegmentCellIntersection> result = new Dictionary<TrajectoryProcessing, SegmentCellIntersection>(new Comparer());
            foreach (KeyValuePair<TrajectoryProcessing, SegmentCellIntersection> item in input)
            {
                SegmentCellIntersection cellIntersection = item.Value;
                TrajectoryProcessing trajectoryProcessing = item.Key;
                double connectionStart = trajectoryProcessing.Trajectory().First().MD;
                double connectionFinal = trajectoryProcessing.Trajectory().Last().MD;

                // check if cell segment is overlapping by compartment interval
                if (cellIntersection.EnteringCell != null && connectionFinal > reservoirStart && connectionStart < reservoirFinal)
                {
                    // excerpt from python code
                    // # Format of tuple in segmentation list
                    // # 0-2 – grid indices i, j, k
                    // #   3 – tag for entering face
                    // #   4 – distance along wellbore to the entering point
                    // #   5 – tag for leaving face
                    // #   6 – distance along wellbore to the leaving point
                    // #   7 – distance along wellbore to the nodal point
                    // #   8 – distance from nodal point to cell center
                    // #   9 – connection factor 
                    // #  10 – wellbore diameter
                    // #  11 – effective Kh
                    // #  12 – penetration direction in relation to principal grid axes
                    // #  13 – effective pressure radius (Peaceman)

                    double intersectionPolylineIndex = cellIntersection.IntersectionInPolylineIndex;
                    Index3 leavingCell = cellIntersection.LeavingCell;
                    Index3 enteringCell = cellIntersection.EnteringCell;
                    Point3 intersectionPoint = cellIntersection.IntersectionPoint;
                    Vector3 surfaceNormal = cellIntersection.SurfaceNormal;
                    CellSide leavingCellSide = cellIntersection.LeavingCellSide;
                    CellSide enteringCellSide = cellIntersection.EnteringCellSide;

                    if (reservoirStart > connectionStart)
                    {
                        TrajectoryProcessing upper = new TrajectoryProcessing();
                        TrajectoryProcessing lower = new TrajectoryProcessing();
                        trajectoryProcessing.Split(reservoirStart, upper, lower);
                        trajectoryProcessing = lower;
                        enteringCellSide = CellSide.None;
                        /*
                        cellIntersection = new SegmentCellIntersection(cellIntersection.IntersectionInPolylineIndex, cellIntersection.LeavingCell,
                            cellIntersection.EnteringCell, cellIntersection.IntersectionPoint, cellIntersection.SurfaceNormal, cellIntersection.LeavingCellSide,
                            cellIntersection.EnteringCellSide);
                        */
                    }
                    if (reservoirFinal < connectionFinal)
                    {
                        TrajectoryProcessing upper = new TrajectoryProcessing();
                        TrajectoryProcessing lower = new TrajectoryProcessing();
                        trajectoryProcessing.Split(reservoirFinal, upper, lower);
                        trajectoryProcessing = upper;
                        leavingCellSide = CellSide.None;
                        /*
                        cellIntersection = new SegmentCellIntersection(cellIntersection.IntersectionInPolylineIndex, cellIntersection.LeavingCell,
                            cellIntersection.EnteringCell, cellIntersection.IntersectionPoint, cellIntersection.SurfaceNormal, cellIntersection.LeavingCellSide,
                            cellIntersection.EnteringCellSide);
                        */
                    }
                    if (trajectoryProcessing != item.Key)
                    {
                        cellIntersection = new SegmentCellIntersection(trajectoryProcessing.Trajectory().First().MD, cellIntersection.LeavingCell,
                            cellIntersection.EnteringCell, cellIntersection.IntersectionPoint, cellIntersection.SurfaceNormal, leavingCellSide,
                            enteringCellSide);
                    }

                    result.Add(trajectoryProcessing, cellIntersection);
                }
            }
            return result.Any()? result: null;
        }

        public Dictionary<TrajectoryProcessing, SegmentCellIntersection> ComputeSegmentation(IEnumerable<SegmentCellIntersection> input)
        {
            Dictionary<TrajectoryProcessing, SegmentCellIntersection> result = new Dictionary<TrajectoryProcessing, SegmentCellIntersection>(new Comparer());
            double[] depthArray = trajectory.ConvertAll<double>(item => item.MD).ToArray();
            IEnumerator<SegmentCellIntersection> intersection = input.GetEnumerator();
            // using (StreamWriter logfile = new StreamWriter("D:\\study\\characterization\\Ocean\\Labs\\Interval\\obj\\ComputeSegmentation.txt"))
            {
                if (intersection.MoveNext())
                {
                    SegmentCellIntersection curr = intersection.Current;
                    int i = (int)curr.IntersectionInPolylineIndex;
                    double t = curr.IntersectionInPolylineIndex - i;
                    double depth = depthArray[i];
                    if (t > double.Epsilon)
                    {
                        double final = depthArray[i + 1];
                        depth += (final - depth) * t;
                    }
                    TrajectoryProcessing upper = new TrajectoryProcessing();
                    TrajectoryProcessing processing = new TrajectoryProcessing();
                    Split(depth, upper, processing);
                    while (intersection.MoveNext())
                    {
                        SegmentCellIntersection next = intersection.Current;
                        i = (int)next.IntersectionInPolylineIndex;
                        t = next.IntersectionInPolylineIndex - i;
                        depth = depthArray[i];
                        if (t > double.Epsilon)
                        {
                            double final = depthArray[i + 1];
                            depth += (final - depth) * t;
                        }
                        upper = new TrajectoryProcessing();
                        TrajectoryProcessing lower = new TrajectoryProcessing();
                        processing.Split(depth, upper, lower);
                        /*
                        if (curr.EnteringCell != null)
                        {
                            Index3 cell = curr.EnteringCell;
                            double start = processing.trajectory[0].MD;
                            logfile.WriteLine($"Adding cell intersection ({cell.I}, {cell.J}, {cell.K}) at interval [{start}, {depth}]");
                        }
                        foreach (TrajectoryPolylineRecord record in upper.Trajectory())
                        {
                            logfile.WriteLine($"\t{record.MD} {record.Point.X} {record.Point.Y} {record.Point.Z}");
                        }
                        */
                        result[upper] = new SegmentCellIntersection(processing.trajectory[0].MD, next.LeavingCell, curr.EnteringCell, curr.IntersectionPoint, curr.SurfaceNormal, next.LeavingCellSide, curr.EnteringCellSide);
                        processing = lower;
                        curr = next;
                    }
                    /*
                    if (curr.EnteringCell != null)
                    {
                        Index3 cell = curr.EnteringCell;
                        double start = processing.trajectory[0].MD;
                        depth = processing.trajectory.Last().MD;
                        logfile.WriteLine($"Adding cell intersection ({cell.I}, {cell.J}, {cell.K}) at interval [{start}, {depth}]");
                    }
                    foreach (TrajectoryPolylineRecord record in processing.Trajectory())
                    {
                        logfile.WriteLine($"\t{record.MD} {record.Point.X} {record.Point.Y} {record.Point.Z}");
                    }
                    */
                    result[processing] = new SegmentCellIntersection(processing.trajectory[0].MD, null, curr.EnteringCell, curr.IntersectionPoint, curr.SurfaceNormal, CellSide.None, curr.EnteringCellSide);
                }
            }
            return result;
        }

        public TrajectoryPolylineRecord? ClosestPoint(Point3 site)
        {
            double ClosestPoint(Segment3 segment)
            {
                Vector3 v = segment.Vector;
                Vector3 w = site - segment.Begin;
                double f = Vector3.Dot(v, w);
                if (f <= 0.0)
                {
                    return 0.0;
                }
                double g = Vector3.Dot(w, w);
                if (g <= f)
                {
                    return 1.0;
                }
                return f / g;
            }
            IEnumerator<TrajectoryPolylineRecord> curr = trajectory.GetEnumerator();
            TrajectoryPolylineRecord? result = null;
            if (curr.MoveNext())
            {
                result = curr.Current;
                double distance = (result?.Point - site).Norm;
                for (IEnumerator<TrajectoryPolylineRecord> next = curr; next.MoveNext(); curr = next)
                {
                    double t = ClosestPoint(new Segment3(curr.Current.Point, next.Current.Point));
                    TrajectoryPolylineRecord f = Interpolate(t, curr.Current, next.Current);
                    double d = (f.Point - site).Norm;
                    if (d < distance)
                    {
                        result = f;
                        distance = d;
                    }
                }
            }
            return result;
        }

        public IPolyline3 ConvertToPolyline3()
        {
            return new Polyline3(trajectory.ConvertAll<Point3>(x => x.Point));
        }

        public static void DeleteBoreholeCollectionContent(BoreholeCollection folder)
        {
            using (ITransaction tr = DataManager.NewTransaction())
            {
                List<BoreholeCollection> bhcList = new List<BoreholeCollection>();
                List<Borehole> bhList = new List<Borehole>();
                bhcList.Add(folder);
                // Traverse the borehole folders (simulate tail recursion)
                int i = 0;
                while (i < bhcList.Count)
                {
                    BoreholeCollection col = bhcList[i++];
                    bhList.AddRange(col);
                    bhcList.AddRange(col.BoreholeCollections);
                }
                foreach (Borehole bh in bhList)
                {
                    tr.Lock(bh);
                    bh.Delete();
                }
                bhcList.Remove(folder);
                foreach (BoreholeCollection bhc in bhcList)
                {
                    tr.Lock(bhc);
                    bhc.Delete();
                }
                tr.Commit();
            }
        }

        public static List<Borehole> GetBoreholesInOneDay(ControlDate controlDate)
        {
            List<WellFlowSet> wfs = new List<WellFlowSet>();
            List<Borehole> result = new List<Borehole>();

            wfs.Add(controlDate.RootWellFlowSet);
            int i = 0;
            while (i < wfs.Count)
            {
                WellFlowSet col = wfs[i++];
                result.AddRange(col.WellFlows.Select(x => x.Borehole));
                wfs.AddRange(col.WellFlowSets);
            }
            return result.Distinct().ToList();
        }

        public static void ClearMarkerCollection(MarkerCollection folder)
        {
            using (ITransaction tran = DataManager.NewTransaction())
            {
                List<Slb.Ocean.Petrel.DomainObject.Well.Horizon> horizonList = new List<Slb.Ocean.Petrel.DomainObject.Well.Horizon>();
                horizonList.AddRange(folder.AllHorizons);
                tran.Lock(folder);
                foreach (Slb.Ocean.Petrel.DomainObject.Well.Horizon horizon in horizonList)
                {
                    tran.Lock(horizon);
                    horizon.Delete();
                }
                List<Slb.Ocean.Petrel.DomainObject.Well.Fault> faultList = new List<Slb.Ocean.Petrel.DomainObject.Well.Fault>();
                faultList.AddRange(folder.Faults);
                foreach (Slb.Ocean.Petrel.DomainObject.Well.Fault fault in faultList)
                {
                    tran.Lock(fault);
                    fault.Delete();
                }
                List<Slb.Ocean.Petrel.DomainObject.Well.Interface> surfaceList = new List<Interface>();
                surfaceList.AddRange(folder.Interfaces);
                foreach (Slb.Ocean.Petrel.DomainObject.Well.Interface surface in surfaceList)
                {
                    tran.Lock(surface);
                    surface.Delete();
                }
                tran.Commit();
            }
        }

        #region SQL Server Data Exchange
        /*
        public class BoreholeData
        {
            public class Dictionary : Dictionary<string, BoreholeData>
            {
                public void Read(SqlConnection conn)
                {
                    SqlCommand cmd = new SqlCommand("SELECT s.Wellbore, s.Borehole, p.Northing, p.Easting, p.Altitude FROM ASSET_TRAJ_SURVEY s, ASSET_SURVEY_LOCATION p WHERE p.LocationId=s.LocationId", conn);
                    using (SqlDataReader dr = cmd.ExecuteReader())
                    {
                        while (dr.Read())
                        {
                            string key = (string)dr["Borehole"];
                            BoreholeData val = new BoreholeData()
                            {
                                Wellbore = (int)dr["Wellbore"],
                                Easting = (double)dr["Easting"],
                                Northing = (double)dr["Northing"],
                                Altitude = (double)dr["Altitude"]
                            };
                            Add(key, val);
                        }
                    }
                }
            }

            public BoreholeData()
            {
                wellbore = 0;
                easting = double.NaN;
                northing = double.NaN;
                altitude = double.NaN;
            }

            private int wellbore;
            private double easting;
            private double northing;
            private double altitude;

            public int Wellbore
            {
                get { return wellbore; }
                set { wellbore = value; }
            }

            public double Easting
            {
                get { return easting; }
                set { easting = value; }
            }

            public double Northing
            {
                get { return northing; }
                set { northing = value; }
            }

            public double Altitude
            {
                get { return altitude; }
                set { altitude = value; }
            }
        }

        public class TrajectoryData
        {
            public class List : List<TrajectoryData>
            {
                public void Read(SqlConnection conn, int wellbore)
                {
                    string query = $"SELECT * from ASSET_TRAJ_INCLINOMETRY WHERE Wellbore={wellbore} ORDER BY MD";
                    // PetrelLogger.InfoOutputWindow(query);
                    SqlCommand cmd = new SqlCommand(query, conn);
                    using (SqlDataReader dr = cmd.ExecuteReader())
                    {
                        while (dr.Read())
                        {
                            TrajectoryData val = new TrajectoryData()
                            {
                                MeasuredDepth = (double)dr["MD"],
                                Inclination = (double)dr["INCL"],
                                AzimuthGN = (double)dr["AZIM_GN"],
                                AzimuthTN = (double)dr["AZIM_TN"],
                                TrueVerticalDepth = (double)dr["TVD"],
                                ZCoord = (double)dr["Z"],
                                XOffset = (double)dr["DX"],
                                YOffset = (double)dr["DY"],
                                XCoord = (double)dr["X"],
                                YCoord = (double)dr["Y"],
                                DogLegSurvey = (double)dr["DLS"]
                            };
                            // PetrelLogger.InfoOutputWindow($"{val.MeasuredDepth}, {val.Inclination}, {val.AzimuthGN}, {val.AzimuthTN}, {val.TrueVerticalDepth}, {val.ZCoord}, {val.XOffset}, {val.YOffset}, {val.XCoord}, {val.YCoord}, {val.DogLegSurvey}");
                            Add(val);
                        }
                    }

                }
                public void WriteCSV(string filename)
                {
                    using (StreamWriter logfile = new StreamWriter(filename))
                    {
                        logfile.WriteLine("MD, INCL, AZIM_GN, AZIM_TN, TVD, Z, DX, DY, X, Y, DLS");
                        foreach (TrajectoryData data in this)
                        {
                            logfile.WriteLine($"{data.MeasuredDepth}, {data.Inclination}, {data.AzimuthGN}, {data.AzimuthTN}, {data.TrueVerticalDepth}, {data.ZCoord}, {data.XOffset}, {data.YOffset}, {data.XCoord}, {data.YCoord}, {data.DogLegSurvey}");
                        }
                    }
                }

                public MDInclinationAzimuthTrajectoryRecord[] ConvertToMDInclinationAzimuthTrajectoryArray()
                {
                    return ConvertAll<MDInclinationAzimuthTrajectoryRecord>(new Converter<TrajectoryData, MDInclinationAzimuthTrajectoryRecord>(TrajectoryData.ConvertToMDInclinationAzimuthTrajectoryRecord)).ToArray();
                }

                public TrajectoryProcessing ConvertToTrajectoryProcessing()
                {
                    return new TrajectoryProcessing(ConvertAll<TrajectoryPolylineRecord>(new Converter<TrajectoryData, TrajectoryPolylineRecord>(TrajectoryData.ConvertToTrajectoryPolylineRecord)));
                }
            }
            public TrajectoryData()
            {
                measured_depth = double.NaN;
                inclination = double.NaN;
                azimuth_gn = double.NaN;
                azimuth_tn = double.NaN;
                true_vertical_depth = double.NaN;
                x_coord = double.NaN;
                y_coord = double.NaN;
                z_coord = double.NaN;
                x_offset = double.NaN;
                y_offset = double.NaN;
                dog_leg_survey = double.NaN;
            }

            private double measured_depth;
            private double inclination;
            private double azimuth_gn;
            private double azimuth_tn;
            private double true_vertical_depth;
            private double x_coord;
            private double y_coord;
            private double z_coord;
            private double x_offset;
            private double y_offset;
            private double dog_leg_survey;

            public double MeasuredDepth
            {
                get { return measured_depth; }
                set { measured_depth = value; }
            }

            public double Inclination
            {
                get { return inclination; }
                set { inclination = value; }
            }

            public double AzimuthGN
            {
                get { return azimuth_gn; }
                set { azimuth_gn = value; }
            }

            public double AzimuthTN
            {
                get { return azimuth_tn; }
                set { azimuth_tn = value; }
            }

            public double TrueVerticalDepth
            {
                get { return true_vertical_depth; }
                set { true_vertical_depth = value; }
            }

            public double XCoord
            {
                get { return x_coord; }
                set { x_coord = value; }
            }

            public double YCoord
            {
                get { return y_coord; }
                set { y_coord = value; }
            }

            public double ZCoord
            {
                get { return z_coord; }
                set { z_coord = value; }
            }

            public double XOffset
            {
                get { return x_offset; }
                set { x_offset = value; }
            }

            public double YOffset
            {
                get { return y_offset; }
                set { y_offset = value; }
            }

            public double DogLegSurvey
            {
                get { return dog_leg_survey; }
                set { dog_leg_survey = value; }
            }

            public static MDInclinationAzimuthTrajectoryRecord ConvertToMDInclinationAzimuthTrajectoryRecord(TrajectoryData data)
            {
                return new MDInclinationAzimuthTrajectoryRecord(data.MeasuredDepth,
                    PetrelUnitSystem.ConvertFromUI(Domain.INCLINATION, data.Inclination), PetrelUnitSystem.ConvertFromUI(Domain.AZIMUTH_GN, data.AzimuthGN));
            }

            public static TrajectoryPolylineRecord ConvertToTrajectoryPolylineRecord(TrajectoryData data)
            {
                return new TrajectoryPolylineRecord(data.MeasuredDepth, data.Inclination, data.AzimuthGN, new Point3(data.XCoord, data.YCoord, data.ZCoord));
            }
        }
        */
        #endregion
    }
}