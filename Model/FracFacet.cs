#undef USEDYNAMICTYPING
#undef MAKESQUAREPLANE

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml;
using System.Threading.Tasks;
using Slb.Ocean.Core;
using Slb.Ocean.Petrel;
using Slb.Ocean.Geometry;
using Slb.Ocean.Basics;
using Slb.Ocean.Petrel.DomainObject;
using Slb.Ocean.Petrel.DomainObject.Shapes;
using Slb.Ocean.Petrel.DomainObject.Well;
using Slb.Ocean.Petrel.DomainObject.Well.Completion;
using Slb.Ocean.Petrel.DomainObject.PillarGrid;
using Slb.Ocean.Petrel.DomainObject.Intersect;
using System.Data;
using System.IO;
// using Slb.Ocean.Petrel.DomainObject.Basics;
// using Slb.Ocean.Petrel.DomainObject.GridModel;

namespace DigitalFrac.Model
{
    public class FracFacet
    {
        // fracture parameters
        private double m_kickoffDepth;
        private double m_downsideHeight;
        private double m_upsideHeight;
        private double m_halfWidth;
        private double m_pillarGap;
        private double m_fracAngle;
        private double m_fracPerm;
        private double m_leftWidth;
        private double m_rightWidth;
        private double m_fracAperture;
        private double m_boreholeDiameter;
        // private int m_numberofSectors;
        private DateTime m_datetimeCompleted;
        private Droid m_completionVersion;
        private Droid m_fracBorehole;
        private Point3 m_fracNode;
        private Facet m_fracFacet;

        public Facet Shape { get { return m_fracFacet; } }
        public Point3 Node { get { return m_fracNode; } }
        public Droid BoreholeInstance { get { return m_fracBorehole; } set { m_fracBorehole = value; } }
        public double KickoffDepth { get { return m_kickoffDepth; } set { m_kickoffDepth = value; } }
        public double FracPerm { get { return m_fracPerm; } set { m_fracPerm = value; } }
        public double Aperture { get { return m_fracAperture; } set { m_fracAperture = value; } }
        public double BoreholeDiameter { get { return m_boreholeDiameter; } set { m_boreholeDiameter = value; } }
        public double DownsideHeight { get { return m_downsideHeight; } set { m_downsideHeight = value; } }
        public double UpsideHeight { get { return m_upsideHeight; } set { m_upsideHeight = value; } }
        public double LeftWidth { get { return m_leftWidth; }  set { m_leftWidth = value; } }
        public double RightWidth { get { return m_rightWidth; } set { m_rightWidth = value; } }
        public double Angle { get { return m_fracAngle; } set { m_fracAngle = value; } }
        public DateTime Completed { get { return m_datetimeCompleted; } set { m_datetimeCompleted = value; } }
        public Droid Version { get { return m_completionVersion; }  set { m_completionVersion = value; } }

        public FracFacet()
        {
            m_kickoffDepth = 0.5 * (3975.0 + 3980.0);
            m_downsideHeight = 60.0;
            m_upsideHeight = 90.0;
            m_halfWidth = 95.0;
            m_leftWidth = 95.0;
            m_rightWidth = 95.0;
            m_pillarGap = 10.0;
            m_fracAngle = -10.0;
            m_fracPerm = 100000.0;
            m_fracAperture = 0.006;
            m_boreholeDiameter = 0.1556;
            m_datetimeCompleted = default(DateTime);
            // m_numberofSectors = 10;
            m_fracBorehole = Droid.Empty;
            m_completionVersion = Droid.Empty;
        }

        private static Borehole GetBorehole(string wellspec)
        {
            string[] spec = wellspec.Split('\\');
            WellRoot root = WellRoot.Get(PetrelProject.PrimaryProject);
            BoreholeCollection folder = root.BoreholeCollection;
            for (int i = 0; i < spec.Length - 1; ++i)
            {
                if (folder.BoreholeCollectionCount == 0)
                {
                    folder = BoreholeCollection.NullObject;
                    break;
                }
                else
                {
                    BoreholeCollection subfolder = folder.BoreholeCollections.FirstOrDefault(item => item.Name.Equals(spec[i], StringComparison.OrdinalIgnoreCase));
                    if (subfolder == BoreholeCollection.NullObject)
                    {
                        folder = BoreholeCollection.NullObject;
                        break;
                    }
                    folder = subfolder;
                }
            }
            if (folder == BoreholeCollection.NullObject)
            {
                PetrelLogger.InfoOutputWindow("PillarCollection.GetBorehole() - cannot locate well folder");
                return Borehole.NullObject;
            }

            /*
            string well = spec.Last();
            Borehole found = Borehole.NullObject;
            foreach (Borehole item in folder)
            {
                if (item.Name == well)
                {
                    found = item;
                    break;
                }
            }
            */

            Borehole found = folder.FirstOrDefault(item => item.Name.Equals(spec.Last(), StringComparison.OrdinalIgnoreCase));
            if (found == Borehole.NullObject)
            {
                PetrelLogger.InfoOutputWindow("PillarCollection.GetBorehole() - cannot locate well instance");
                return Borehole.NullObject;
            }
            return found;
        }

        BoreholeCollection ResetOutput(string outputCollection)
        {
            WellRoot root = WellRoot.Get(PetrelProject.PrimaryProject);
            BoreholeCollection storage = root.BoreholeCollection.BoreholeCollections.FirstOrDefault(item => item.Name == outputCollection);
            if (storage == BoreholeCollection.NullObject)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(root.BoreholeCollection);
                    storage = root.BoreholeCollection.CreateBoreholeCollection(outputCollection);
                    transaction.Commit();
                }
            }
            else
            {
                TrajectoryProcessing.DeleteBoreholeCollectionContent(storage);
            }
            return storage;
        }

        public void DefineFracture_UnstructuredGrid(string wellspec)
        {
            // 1. Find kik-off point at specific wellname
            Borehole found = GetBorehole(wellspec);
            if (found == Borehole.NullObject)
            {
                PetrelLogger.InfoOutputWindow("cannot locate well instance");
                return;
            }

            PetrelLogger.InfoOutputWindow($"processing borehole {wellspec}");
            TrajectoryProcessing traj = new TrajectoryProcessing(found.Trajectory.TrajectoryPolylineRecords);
            TrajectoryProcessing upperPart = new TrajectoryProcessing();
            TrajectoryProcessing lowerPart = new TrajectoryProcessing();
            traj.Split(m_kickoffDepth, upperPart, lowerPart);
            //PetrelLogger.InfoOutputWindow("content of upper part of trajectory:");
            //foreach (TrajectoryPolylineRecord rec in upperPart.Trajectory()) { PetrelLogger.InfoOutputWindow($"\t{rec.MD} {rec.Point.X} {rec.Point.Y} {rec.Point.Z}"); }
            //PetrelLogger.InfoOutputWindow("content of lower part of trajectory:");
            //foreach (TrajectoryPolylineRecord rec in lowerPart.Trajectory()) { PetrelLogger.InfoOutputWindow($"\t{rec.MD} {rec.Point.X} {rec.Point.Y} {rec.Point.Z}"); }

            // 2. Create folder for well output or clear its content if exists
            BoreholeCollection storage = ResetOutput("Ocean Labs");

            // 3. create patch of wellbores representing fracture plane
            using (ITransaction transaction = DataManager.NewTransaction())
            {
                Point3 origin = upperPart.Trajectory().Last().Point;
                transaction.Lock(storage);
                int num = (int)Math.Round(2.0 * m_halfWidth / m_pillarGap + 0.5, MidpointRounding.AwayFromZero);
                double a = (90.0 - m_fracAngle) * System.Math.PI / 180.0;
                double cosa = Math.Cos(a);
                double sina = Math.Sin(a);
                for (int i = 0; i < num; ++i)
                {
                    double x = 2.0 * m_halfWidth * i / (num - 1) - m_halfWidth;
                    double y = sina * x;
                    x = cosa * x;
                    Point3 start = new Point3(origin.X + x, origin.Y + y, origin.Z + m_upsideHeight);
                    Point3 final = new Point3(origin.X + x, origin.Y + y, origin.Z - m_downsideHeight);

                    Borehole well = storage.CreateBorehole($"{i + 1:0000}");
                    well.WellHead = new Point2(start.X, start.Y);
                    var kb = new ReferenceLevel("Origin", start.Z, "Fracture Origin");
                    well.ReferenceLevels = new List<ReferenceLevel> { kb };
                    well.WorkingReferenceLevel = kb;

                    var tc = well.Trajectory.TrajectoryCollection;
                    transaction.Lock(tc);
                    var tr = tc.CreateTrajectory<XyzTrajectory>("Fracture Pillar");
                    tr.Records = new[] { new XyzTrajectoryRecord(start), new XyzTrajectoryRecord(final) };
                    tr.Settings = new XyzTrajectorySettings(CalculationAlgorithmType.Linearization);
                    tc.DefinitiveSurvey = tr;
                }
                transaction.Commit();
            }

            // 4. apply unstructured gridding for this construct
        }

        private XmlDocument Calculate(Borehole borehole, Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid grid, Facet facet)
        {
            XmlDocument xmlDocument = new XmlDocument();

            IPillarGridIntersectionService pillarGridIntersectionService = CoreSystem.GetService<IPillarGridIntersectionService>();

            XmlElement root = xmlDocument.CreateElement("Borehole");
            xmlDocument.AppendChild(root);
            XmlAttribute attribute = xmlDocument.CreateAttribute("name");
            attribute.Value = borehole.Name;
            root.Attributes.Append(attribute);

            //XmlElement facetElement = xmlDocument.CreateElement("Facet");
            //root.AppendChild(facetElement);
            //...

            IEnumerable<FacetCellIntersection> intersectionCells = pillarGridIntersectionService.GetPillarGridPlaneIntersection(grid, facet);
            PetrelLogger.InfoOutputWindow($"number of intersection cells = {intersectionCells.Count()}");
            foreach (FacetCellIntersection cellIntersection in intersectionCells)
            {
                XmlElement cellElement = xmlDocument.CreateElement("Cell");
                root.AppendChild(cellElement);
                attribute = xmlDocument.CreateAttribute("index");
                attribute.Value = cellIntersection.CellIndex.ToString();
                cellElement.Attributes.Append(attribute);

                XmlElement points = xmlDocument.CreateElement("Polygon");
                cellElement.AppendChild(points);

                foreach (Point3 p in cellIntersection.Points)
                {
                    XmlElement vertex = xmlDocument.CreateElement("Vertex");
                    vertex.InnerText = p.ToString();
                    points.AppendChild(vertex);
                }

                /*
                XmlElement horizons = xmlDocument.CreateElement("Horizons");
                cellElement.AppendChild(horizons);

                foreach (Slb.Ocean.Petrel.DomainObject.PillarGrid.Horizon h in cellIntersection.Horizons)
                {
                    XmlElement horizon = xmlDocument.CreateElement("Horizon");
                    horizon.InnerText = h.Name;
                    horizons.AppendChild(horizon);
                }
                */

                /*
                XmlElement edges = null;

                for (int v0 = 0, v1 = 1; v0 < cellIntersection.Points.Length; v0++, v1++)
                {
                    Point3 startPoint = cellIntersection.Points[v0];
                    if (v1 == cellIntersection.Points.Length)
                    {
                        v1 = 0;
                    }

                    // get only the horizontal edges
                    Slb.Ocean.Petrel.DomainObject.PillarGrid.Horizon intersectedHorizon = cellIntersection.Horizons[v0];
                    if (intersectedHorizon == Slb.Ocean.Petrel.DomainObject.PillarGrid.Horizon.NullObject || intersectedHorizon != cellIntersection.Horizons[v1])
                    {
                        continue;
                    }

                    Point3 endPoint = cellIntersection.Points[v1];
                    Vector3 vector = endPoint - startPoint;

                    double dip = vector.Dip.Degrees;
                    double azimuth = vector.Azimuth.Degrees;
                    if (azimuth < 0)
                    {
                        azimuth += 360;
                    }

                    if (edges == null)
                    {
                        XmlElement cellElement = xmlDocument.CreateElement("Cell");
                        root.AppendChild(cellElement);
                        attribute = xmlDocument.CreateAttribute("index");
                        attribute.Value = cellIntersection.CellIndex.ToString();
                        cellElement.Attributes.Append(attribute);

                        edges = xmlDocument.CreateElement("IntersectionEdges");
                        cellElement.AppendChild(edges);
                    }

                    XmlElement edge = xmlDocument.CreateElement("Edge");
                    edges.AppendChild(edge);

                    XmlElement dipElement = xmlDocument.CreateElement("Dip");
                    dipElement.InnerText = dip.ToString();
                    edge.AppendChild(dipElement);

                    XmlElement azimuthElement = xmlDocument.CreateElement("Azimuth");
                    azimuthElement.InnerText = azimuth.ToString();
                    edge.AppendChild(azimuthElement);

                    XmlElement horizon = xmlDocument.CreateElement("IntersectedHorizon");
                    attribute = xmlDocument.CreateAttribute("name");
                    attribute.Value = intersectedHorizon.Name;
                    horizon.Attributes.Append(attribute);

                    attribute = xmlDocument.CreateAttribute("K");
                    attribute.Value = intersectedHorizon.K.ToString();
                    horizon.Attributes.Append(attribute);
                    edge.AppendChild(horizon);
                }
                */
            }

            return xmlDocument;

        }

        private static PointProperty GetOrCreateWellknownProperty(PointSet ptSet, PointSetPropertyType propType)
        {
            PointProperty pointProperty = PointProperty.NullObject;
            if (ptSet.HasWellKnownProperty(propType))
                return ptSet.GetWellKnownProperty(propType);
            using (ITransaction tr = DataManager.NewTransaction())
            {
                tr.Lock(ptSet);
                pointProperty = ptSet.CreateWellKnownProperty(propType);
                tr.Commit();
            }
            return pointProperty;
        }

        private void MakeConnection(Facet facet, IEnumerable<FacetCellIntersection> cells, Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid grid)
        {
            GridProcessing gridProcessing = new GridProcessing(grid, "PERMXY", "PERMXY", "PERMZ", "NTG");
            Property ntg = grid.Properties.FirstOrDefault(item => item.Name.Equals("NTG", StringComparison.OrdinalIgnoreCase));
            Property horPerm = grid.Properties.FirstOrDefault(item => item.Name.Equals("PERMXY", StringComparison.OrdinalIgnoreCase));
            Property verPerm = grid.Properties.FirstOrDefault(item => item.Name.Equals("PERMZ", StringComparison.OrdinalIgnoreCase));
            Index3 numCells = grid.NumCellsIJK;
            int IndexToNaturalOrder(Index3 index) => index.I + numCells.I * (index.J + numCells.J * index.K);
            Index3 NaturalOrderToIndex(int num) => new Index3(num % numCells.I, (num % (numCells.I * numCells.J)) / numCells.I, num / (numCells.I * numCells.J));

            // affine transform from 3D world coordinates system to 2D local coordinate system
            Matrix4 T = new Matrix4();
            T.Translate(-facet.Plane.DefiningPoint.X, -facet.Plane.DefiningPoint.Y, -facet.Plane.DefiningPoint.Z);
            Vector3 Oy = new Vector3(0.0, 1.0, 0.0);
            double a = Math.Acos(Vector3.Dot(Oy, facet.Plane.Normal.NormalizedVector));
            T.Rotate(0.0, 0.0, -a, false);

            IEnumerable<Collection> folders = PetrelProject.PrimaryProject.Collections;
            string storageName = "Ocean Labs";
            Collection storage = PetrelProject.PrimaryProject.Collections.FirstOrDefault(item => item.Name.Equals(storageName, StringComparison.OrdinalIgnoreCase));
            if (Collection.NullObject == storage)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    Project project = PetrelProject.PrimaryProject;
                    transaction.Lock(project);
                    storage = project.CreateCollection(storageName);
                    transaction.Commit();
                }
            }
            string siteName = "Bounding Points";
            PointSet site = storage.PointSets.FirstOrDefault(item => item.Name.Equals(siteName, StringComparison.OrdinalIgnoreCase));
            if (PointSet.NullObject != site)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(storage);
                    storage.Remove(site);
                    transaction.Commit();
                }
            }
            using (ITransaction transaction = DataManager.NewTransaction())
            {
                transaction.Lock(storage);
                site = storage.CreatePointSet(siteName);
                transaction.Commit();
            }

            PointProperty dipAzimuth = GetOrCreateWellknownProperty(site, WellKnownPointSetPropertyTypes.DipAzimuth);
            PointProperty dipAngle = GetOrCreateWellknownProperty(site, WellKnownPointSetPropertyTypes.DipAngle);

            using (ITransaction transaction = DataManager.NewTransaction())
            {
                transaction.Lock(site);
                transaction.Lock(dipAzimuth);
                transaction.Lock(dipAngle);

                List<Point3> points = new List<Point3>();
                List<double> valAzimuth = new List<double>();
                List<double> valAngle = new List<double>();

                Vector3 q = facet.Plane.Normal.NormalizedVector;
                points.Add(facet.Plane.DefiningPoint);
                valAzimuth.Add(q.Azimuth.Radians);
                valAngle.Add(q.Dip.Radians);

                foreach (Plane3 p in facet.BoundingPlanes)
                {
                    q = p.Normal.NormalizedVector;
                    points.Add(p.DefiningPoint);
                    valAzimuth.Add(q.Azimuth.Radians);
                    valAngle.Add(q.Dip.Radians);
                }

                site.Points = new Point3Set(points);
                dipAzimuth.SetRecordValues<double>(dipAzimuth.Records, valAzimuth);
                dipAngle.SetRecordValues<double>(dipAngle.Records, valAngle);

                transaction.Commit();
            }

            string fieldName = "Bounding Direction";
            PolylineSet fieldSegment = storage.PolylineSets.FirstOrDefault(item => item.Name.Equals(fieldName, StringComparison.OrdinalIgnoreCase));
            using (ITransaction transaction = DataManager.NewTransaction())
            {
                transaction.Lock(storage);
                if (PolylineSet.NullObject != fieldSegment)
                {
                    transaction.Lock(fieldSegment);
                    storage.Remove(fieldSegment);
                }
                fieldSegment = storage.CreatePolylineSet(fieldName);

                List<Polyline3> pos = new List<Polyline3>();
                foreach (Plane3 s in facet.BoundingPlanes)
                {
                    Point3[] p = new Point3[2];
                    p[0] = s.DefiningPoint;
                    p[1] = s.DefiningPoint + s.Normal.NormalizedVector  * 10.0;
                    pos.Add(new Polyline3(p));
                }
                fieldSegment.Polylines = pos.ToArray<IPolyline3>();
                transaction.Commit();
            }

            IntersectRoot ixRoot;
            ixRoot = IntersectRoot.Get(PetrelProject.PrimaryProject);
            UserEditCollection topLevelUserEditColl;
            if (ixRoot.HasUserEditCollection)
            {
                topLevelUserEditColl = ixRoot.UserEditCollection;
            }
            else
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(PetrelProject.PrimaryProject);
                    topLevelUserEditColl = ixRoot.CreateUserEditCollection();
                    transaction.Commit();
                }
            }

            UserEditCollection oceanLabUserEditColl = topLevelUserEditColl.UserEditCollections.FirstOrDefault(item => item.Name.Equals("Ocean Labs", StringComparison.OrdinalIgnoreCase));
            using (ITransaction transaction = DataManager.NewTransaction())
            {
                // create Ocean Labs collection in INTERSECT User Edit
                if (oceanLabUserEditColl == null)
                {
                    transaction.Lock(topLevelUserEditColl);
                    oceanLabUserEditColl = topLevelUserEditColl.CreateUserEditCollection("Ocean Labs");
                }
                // clear Ocean Labs collection and child collections
                else
                {
                    transaction.Lock(oceanLabUserEditColl);
                    // simulate recursive processing with Stack
                    Stack<UserEditCollection> child = new Stack<UserEditCollection>();
                    child.Push(oceanLabUserEditColl);
                    while (child.Count > 0)
                    {
                        UserEditCollection parent = child.Pop();
                        transaction.Lock(parent);
                        foreach (FieldManagementUserEdit fieldManagementUserEdit in parent.FieldManagementUserEdits)
                        {
                            transaction.Lock(fieldManagementUserEdit);
                            fieldManagementUserEdit.Delete();
                        }
                        foreach (ReservoirUserEdit reservoirUserEdit in parent.ReservoirUserEdits)
                        {
                            transaction.Lock(reservoirUserEdit);
                            reservoirUserEdit.Delete();
                        }
                        foreach (UserEditCollection userEditCollection in parent.UserEditCollections)
                        {
                            child.Push(userEditCollection);
                        }
                    }
                }
                transaction.Commit();
            }

            fieldName = "Intersected Cells";
            fieldSegment = storage.PolylineSets.FirstOrDefault(item => item.Name.Equals(fieldName, StringComparison.OrdinalIgnoreCase));
            List<FacetCellIntersection> selectedFacet = new List<FacetCellIntersection>();
            using (ITransaction transaction = DataManager.NewTransaction())
            {
                transaction.Lock(storage);
                if (PolylineSet.NullObject != fieldSegment)
                {
                    transaction.Lock(fieldSegment);
                    storage.Remove(fieldSegment);
                }
                fieldSegment = storage.CreatePolylineSet(fieldName);

                List<Polyline3> seg = new List<Polyline3>();
                foreach(FacetCellIntersection intersection in cells)
                {
                    if (intersection.Points.Length > 2)
                    {
                        seg.Add(new Polyline3(intersection.Points, true));
                        selectedFacet.Add(intersection);
                    }
                }
                fieldSegment.Polylines = seg.ToArray<IPolyline3>();
                transaction.Commit();
            }

            PolylinePropertyCollection propColl = fieldSegment.PropertyCollection;
            if (propColl == null)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(fieldSegment);
                    propColl = fieldSegment.CreatePropertyCollection();
                    transaction.Commit();
                }
            }

            PolylinePropertyType areaType = WellKnownPolylinePropertyTypes.Area;
            PolylineProperty areaProp = PolylineProperty.NullObject;
            if (propColl.HasWellKnownProperty(areaType))
            {
                areaProp = propColl.GetWellKnownProperty(areaType);
            }
            else
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(propColl);
                    areaProp = propColl.CreateWellKnownProperty(areaType);
                    transaction.Commit();
                }
            }

            string indexName = "natural order";
            DictionaryPolylineProperty indexProp = propColl.DictionaryProperties.FirstOrDefault(item => item.Name.Equals(indexName, StringComparison.OrdinalIgnoreCase));
            if (DictionaryPolylineProperty.NullObject == indexProp)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(propColl);                    
                    // indexProp = propColl.CreateDictionaryProperty(typeof(Int32), indexName);
                    indexProp = propColl.CreateDictionaryProperty(PetrelProject.WellKnownTemplates.DiscreteFracturePropertyGroup.FracturePatchSet, indexName);
                    transaction.Commit();
                }
            }

            string labelName = "cell index";
            DictionaryPolylineProperty labelProp = propColl.DictionaryProperties.FirstOrDefault(item => item.Name.Equals(labelName, StringComparison.OrdinalIgnoreCase));
            if (DictionaryPolylineProperty.NullObject == labelProp)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(propColl);
                    // indexProp = propColl.CreateDictionaryProperty(typeof(Int32), indexName);
                    labelProp = propColl.CreateDictionaryProperty(typeof(string), labelName);
                    transaction.Commit();
                }
            }

            PolylineProperty GetOrCreatePolylineProperty(Template t, string name)
            {
                PolylineProperty prop = propColl.Properties.FirstOrDefault(item => item.Name.Equals(name, StringComparison.OrdinalIgnoreCase));
                if (PolylineProperty.NullObject == prop)
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        transaction.Lock(propColl);
                        prop = propColl.CreateProperty(t, name);
                        transaction.Commit();
                    }
                }
                return prop;
            }

            PolylineProperty propHeight = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.GeometricalGroup.CellDeltaZ, "cell height");

            //GridProcessing.CircularDrainageZ circularDrainage = new GridProcessing.CircularDrainageZ(gridProcessing);
            //PolylineProperty propCircular = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "circular drainage"); 

            SquareDrainageZ squareDrainage = new SquareDrainageZ(gridProcessing);
            PolylineProperty propSquare = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "square drainage");

            BlockPressureEquivalentZ blockDrainage = new BlockPressureEquivalentZ(gridProcessing, gridProcessing);
            PolylineProperty propBlock = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "block radius");
            PolylineProperty propLength = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "wing length"); 
            PolylineProperty propPenetrationRatio = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.MiscellaneousGroup.Fraction, "penetration ratio"); 
            PolylineProperty propConductivity = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.MiscellaneousGroup.Fraction, "fracture conductivity"); 
            PolylineProperty propProppantNumber = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.MiscellaneousGroup.Fraction, "proppant number"); 
            PolylineProperty propEquivalentRadius = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "equivalent radius"); 
            PolylineProperty propPseudoSkin = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "pseudo-skin"); 
            PolylineProperty propBlockT = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "grid block term"); 
            PolylineProperty propFracT = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "frac plane term"); 
            PolylineProperty propTransmissibility = GetOrCreatePolylineProperty(PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "transmissibility"); 

            using (ITransaction transaction = DataManager.NewTransaction())
            {
                string wellName = "264_ACh_completed";

                transaction.Lock(fieldSegment);
                transaction.Lock(propColl);
                transaction.Lock(oceanLabUserEditColl);
                ReservoirUserEdit wellConnection = oceanLabUserEditColl.CreateReservoirUserEdit(wellName);
                transaction.Lock(wellConnection);

                StringBuilder stringBuilder = new StringBuilder();
                stringBuilder.AppendLine("START");
                stringBuilder.AppendLine("DATE \"14-Dec-2024 13:00:00\"");
                stringBuilder.AppendLine("#TIME 0.0");
                stringBuilder.AppendLine();
                stringBuilder.AppendLine("WellDef \"" + wellName + ";Tubing 1\" {");
                stringBuilder.AppendLine("    Undefined=\"False\"");
                stringBuilder.AppendLine();

                int[] fieldSize = new int[] { 20, 39-20, 54-39, 64-54, 85-64, 102-85, 120-102, 128-120, 144-128, 172-144, 197-172, 218-197, 242-218 };
                DataTable dataTable = new DataTable();
                dataTable.Columns.Add("Cell", typeof(string));
                dataTable.Columns.Add("Completion", typeof(string));
                dataTable.Columns.Add("SegmentNode", typeof(int));
                dataTable.Columns.Add("Status", typeof(string));
                dataTable.Columns.Add("TrueVerticalDepth", typeof(double));
                dataTable.Columns.Add("MeasuredDepth", typeof(double));
                dataTable.Columns.Add("WellBoreRadius", typeof(double));
                dataTable.Columns.Add("Skin", typeof(double));
                dataTable.Columns.Add("PiMultiplier", typeof(double));
                dataTable.Columns.Add("PressureEquivalentRadius", typeof(double));
                dataTable.Columns.Add("PermeabilityThickness", typeof(double));
                dataTable.Columns.Add("Transmissibility", typeof(double));
                dataTable.Columns.Add("PenetrationDirection", typeof(string));

                IEnumerator<FacetCellIntersection> facetEnum = selectedFacet.GetEnumerator();
                foreach (Polyline p in fieldSegment.GetPolylines())
                {
                    Slb.Ocean.Petrel.DomainObject.Basics.IPropertyAccess pa = p.PropertyAccess;
                    Point3[] pos3 = p.ToArray();
                    Index3 cell = null;
                    if (facetEnum.MoveNext())
                    {
                        FacetCellIntersection fci = facetEnum.Current;
                        if (pos3.SequenceEqual(fci.Points))
                        {
                            cell = fci.CellIndex;
                        }
                        else
                        {
                            PetrelLogger.InfoOutputWindow("cannot assign corresponding index in fracture patch set");
                        }
                    }
                    T.TransformPoints(pos3);
                    Point2[] pos2 = pos3.AsEnumerable().Select(item => new Point2(item.X, item.Z)).ToArray();
                    // apply shoelace formula to compute facet area
                    Point2 prev = pos2.Last();
                    double area = 0.0;
                    foreach (Point2 curr in pos2)
                    {
                        area += curr.X * prev.Y - prev.X * curr.Y;
                        prev = curr;
                    }
                    area *= 0.5;
                    Point2 centerPos = new Point2(pos2.Average(item => item.X), pos2.Average(item => item.Y));
                    pa.SetPropertyValue(areaProp, area);
                    if (cell != null)
                    {
                        Index3 modelCell = ModelingUnitSystem.ConvertIndexToUI(grid, cell);
                        pa.SetPropertyValue(labelProp, modelCell.ToString());
                        Int32 naturalOrder = IndexToNaturalOrder(cell);
                        pa.SetPropertyValue(indexProp, naturalOrder);
                        double dz = gridProcessing.Dz(cell);
                        pa.SetPropertyValue(propHeight, dz);
                        double xf = 0.5 * area / dz; // fracture wing length
                        if (ntg[cell] > 0.0) // set active cells only
                        {
                            double xe = 2.0 * squareDrainage.Radius(cell);
                            double blockRadius = blockDrainage.Radius(cell);
                            //pa.SetPropertyValue(propCircular, circularDrainage.Radius(cell));
                            pa.SetPropertyValue(propSquare, xe);
                            pa.SetPropertyValue(propBlock, blockRadius);
                            pa.SetPropertyValue(propLength, xf);
                            double Ix = 2.0 * xf / xe;
                            pa.SetPropertyValue(propPenetrationRatio, Ix);
                            double k = gridProcessing.Kz_GeometricMean(cell);
                            double Cfd = m_fracPerm * m_fracAperture / (k * xf);
                            pa.SetPropertyValue(propConductivity, Cfd);
                            double Nprop = Ix * Ix * Cfd;
                            pa.SetPropertyValue(propProppantNumber, Nprop);
                            double u = Math.Log(Cfd);
                            double f = ((0.116 * u - 0.328) * u + 1.65) / (((0.005 * u + 0.064) * u + 0.18) * u + 1.0);
                            double r = xf * Math.Exp(-f);
                            pa.SetPropertyValue(propEquivalentRadius, r);
                            double s = Math.Log(0.5 * m_boreholeDiameter / r);
                            pa.SetPropertyValue(propPseudoSkin, s);
                            // double blockT = 0.008527 * 2.0 * Math.PI * k * dz / (Math.Log(blockRadius / (0.5 * m_boreholeDiameter)) + s);
                            //if (blockT < 0.0)
                            //{
                            //    PetrelLogger.InfoOutputWindow($"negative well-grid block transmissibility {t}");
                            //}
                            // compute transmissibility term inside the grid block of hydrodynamic model
                            double blockT = 2.0 * Math.PI * k * dz / (Math.Log(xe / m_boreholeDiameter) + s);
                            pa.SetPropertyValue(propBlockT, blockT);
                            // compute transmissibility term inside fracture plane
                            double fracT = 0.0;
                            prev = pos2.Last();
                            foreach (Point2 curr in pos2)
                            {
                                Vector2 tangent  = curr - prev;
                                Point2 origin = prev + 0.5 * tangent;
                                Vector2 q = new Vector2(-origin.X, -origin.Y);
                                Vector2 d = tangent.ToNormalized();
                                Vector2 n = d * Vector2.Dot(d, q) - q;
                                Vector2 A = n.ToNormalized() * tangent.Norm * m_fracAperture * m_fracPerm;
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
                            pa.SetPropertyValue(propFracT, fracT);
                            const double c = 0.008527;
                            double t = c * blockT;
                            if (fracT > 0.0)
                            {
                                t = c / (1.0 / blockT + 1.0 / fracT);
                            }
                            else
                            {
                                PetrelLogger.InfoOutputWindow($"fracture kick-off point is inside cell {modelCell}");
                            }
                            pa.SetPropertyValue(propTransmissibility, t);
                            dataTable.Rows.Add($"({modelCell.I} {modelCell.J} {modelCell.K})", "\"Frac 1\"", 1, "OPEN", -facet.Plane.DefiningPoint.Z, m_kickoffDepth, m_boreholeDiameter * 0.5, 0.0, 1.0, blockRadius, k * dz, t, "K");
                        }
                    }
                }

                //stringBuilder.AppendLine("    WellToCellConnections [");
                //foreach (DataColumn column in dataTable.Columns)
                //{
                //    stringBuilder.Append(column.ColumnName.PadLeft(fieldSize[column.Ordinal]));
                //}
                //stringBuilder.AppendLine();
                //foreach (DataRow row in dataTable.Rows)
                //{
                //    foreach (DataColumn column in dataTable.Columns)
                //    {
                //        stringBuilder.Append(row[column].ToString().PadLeft(fieldSize[column.Ordinal]));
                //    }
                //    stringBuilder.AppendLine();
                //}
                //stringBuilder.AppendLine("    ]");

                stringBuilder.AppendLine("    WellToCellConnections {");
                foreach (DataColumn column in dataTable.Columns)
                {
                    StringBuilder property = new StringBuilder();
                    property.Append($"        {column.ColumnName} = [ ADD");
                    foreach (DataRow row in dataTable.Rows)
                    {
                        if (property.Length > 80)
                        {
                            stringBuilder.AppendLine(property.ToString());
                            property.Clear();
                            property.Append("            ");
                        }
                        property.Append($" {row[column].ToString()}");
                    }
                    stringBuilder.AppendLine(property.ToString());
                    stringBuilder.AppendLine("        ]");
                }
                stringBuilder.AppendLine("    }");

                stringBuilder.AppendLine("}");
                wellConnection.Text = stringBuilder.ToString();
                transaction.Commit();
            }

            Dictionary<int, Point2[]> vertex = new Dictionary<int, Point2[]>();
            Dictionary<int, Point2> center = new Dictionary<int, Point2>();
            if (Property.NullObject != ntg && Property.NullObject != horPerm && Property.NullObject != verPerm)
            {
                foreach (FacetCellIntersection intersection in cells)
                {
                    if (intersection.Points.Length > 2 && ntg[intersection.CellIndex] > 0.0f)
                    {
                        Point3[] pos3 = (Point3[])intersection.Points.Clone();
                        T.TransformPoints(pos3);
                        int s = IndexToNaturalOrder(intersection.CellIndex);
                        Point2[] pos2 = pos3.AsEnumerable().Select(item => new Point2(item.X, item.Z)).ToArray();
                        vertex.Add(s, pos2);
                        center.Add(s, new Point2(pos2.Average(p => p.X), pos2.Average(p => p.Y)));
                    }
                }
                // array of active cells sorted in natural order
                int[] ss = center.Keys.ToArray();
                Array.Sort(ss);
                int ns = ss.Length;
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(oceanLabUserEditColl);
                    ReservoirUserEdit fractureConnection = oceanLabUserEditColl.CreateReservoirUserEdit("Fracture Connection");
                    transaction.Lock(fractureConnection);

                    StringBuilder adjacentString = new StringBuilder();
                    adjacentString.AppendLine("MODEL_DEFINITION");
                    adjacentString.AppendLine();
                    adjacentString.AppendLine("CellSelectionFamily \"FractureCells\" {");
                    adjacentString.AppendLine("    SelectionNames=[");
                    for (int i = 0; i < ns; ++i)
                    {
                        //Index3 cell = NaturalOrderToIndex(ss[i]);
                        //adjacentString.AppendLine($"        \"CELL_{cell.I}_{cell.J}_{cell.K}\"");
                        adjacentString.AppendLine($"        \"S{i + 1}\"");
                    }
                    adjacentString.AppendLine("    ]");
                    adjacentString.AppendLine("    Cells=[");
                    for (int i = 0; i < ns; ++i)
                    {
                        adjacentString.AppendLine($"        [{ss[i]}]");
                    }
                    adjacentString.AppendLine("    ]");
                    adjacentString.AppendLine("}");
                    adjacentString.AppendLine();
                    adjacentString.AppendLine("CellPropertyEdit \"AdjacentConnection\" [");
                    adjacentString.AppendLine("    Family          Name           Property          Expression");

                    StringBuilder distantString = new StringBuilder();
                    distantString.AppendLine("ConnectionSet \"DistantConnection\" [");
                    distantString.AppendLine("  Cell1       Cell2        Transmissibility");
                    // quick and dirty implementation of connectivity computation
                    for (int i = 0; i < ns; ++i)
                    {
                        Index3 source = NaturalOrderToIndex(ss[i]);
                        for (int j = i + 1; j < ns; ++j)
                        {
                            Index3 target = NaturalOrderToIndex(ss[j]);


                            double AdjasentCellTransmissibility(CellCorner[] sourceCorners, CellCorner[] targetCorners, Property dirPerm, Property dirNtg = null)
                            {
                                Point3[] sourceFace = grid.GetCellCorners(source, sourceCorners);
                                Point3[] targetFace = grid.GetCellCorners(target, targetCorners);
                                //
                                // it is supposed here that sourceFace is the same as targetFace
                                if (!sourceFace.SequenceEqual(targetFace))
                                {
                                    PetrelLogger.Warn("precondition about equality of right face of source cell and left face of target face is violated");
                                }
                                //TODO: compute overlapping geometry for sourceFace and targetFace for general case
                                Point3 sourceCenter = grid.GetCellCenter(source);
                                Point3 faceCenter = new Point3(sourceFace.Average(p => p.X), sourceFace.Average(p => p.Y), sourceFace.Average(p => p.Z));
                                Point3 prev = sourceFace.Last();
                                Vector3 D = faceCenter - sourceCenter;
                                Vector3 sumA = new Vector3(Vector3.Zero);
                                foreach (Point3 curr in sourceFace)
                                {
                                    Vector3 A = Vector3.Cross(curr - faceCenter, prev - faceCenter);
                                    if (Vector3.Dot(D, A) < 0.0)
                                    {
                                        A = Vector3.Negate(A);
                                    }
                                    sumA += A;
                                    prev = curr;
                                }
                                double sourcePerm = PetrelUnitSystem.ConvertToUI(dirPerm.Template, dirPerm[source]);
                                double sourceNtg = dirNtg is null ? 1.0 : PetrelUnitSystem.ConvertToUI(dirNtg.Template, dirNtg[source]);
                                double sourceAD = Vector3.Dot(D, sumA);
                                double sourceD2 = Vector3.Dot(D, D);
                                double sourceT = sourcePerm * sourceNtg * sourceAD / sourceD2;
                                //
                                // compute transmissibility from source facet to target node
                                Point3 targetCenter = grid.GetCellCenter(target);
                                faceCenter = new Point3(targetFace.Average(p => p.X), targetFace.Average(p => p.Y), targetFace.Average(p => p.Z));
                                prev = targetFace.Last();
                                D = targetCenter - faceCenter;
                                sumA = new Vector3(Vector3.Zero);
                                foreach (Point3 curr in targetFace)
                                {
                                    Vector3 A = Vector3.Cross(curr - faceCenter, prev - faceCenter);
                                    if (Vector3.Dot(D, A) < 0.0)
                                    {
                                        A = Vector3.Negate(A);
                                    }
                                    sumA += A;
                                    prev = curr;
                                }
                                double targetPerm = PetrelUnitSystem.ConvertToUI(dirPerm.Template, dirPerm[target]);
                                double targetNtg = dirNtg is null ? 1.0 : PetrelUnitSystem.ConvertToUI(dirNtg.Template, dirNtg[target]);
                                double targetAD = Vector3.Dot(D, sumA);
                                double targetD2 = Vector3.Dot(D, D);
                                double targetT = targetPerm * targetNtg * targetAD / targetD2;
                                //
                                // compute resulting transmissibility
                                return 0.008527 / (1.0 / sourceT + 1.0 / targetT);
                            }

                            double FractureTransmissibility(int sourceFacet, int targetFacet, double sourcePerm, double targetPerm)
                            {
                                double w_frac = m_fracAperture;

                                // compute non-neighbour connection transmissibilities
                                // according to following formula
                                //
                                //           CDARCY * TMULTX_i
                                // TRANX_i = --------------------
                                //             1 / T_i + 1 / T_j
                                //
                                //                    A * D_i
                                // T_i = PERMX * NTG ----------
                                //                    D_i * D_i
                                //
                                // (A * D_i) = A_x * D_ix + A_y * D_iy + A_z * D_iz
                                //
                                // (D_i * D_i) = D_ix^2 + D_iy^2 + D_iz^2
                                //
                                // compute transmissibility from source facet to target facet
                                double sourceT = 0.0;
                                Point2[] pos = vertex[targetFacet];
                                Point2 prev = pos.Last();
                                foreach (Point2 curr in pos)
                                {
                                    Vector2 t = curr - prev;
                                    Point2 p = prev + 0.5 * t;
                                    Vector2 q = center[targetFacet] - p;
                                    Vector2 s = t.ToNormalized();
                                    Vector2 n = s * Vector2.Dot(s, q) - q;
                                    Vector2 A = n.ToNormalized() * t.Norm * w_frac;
                                    // check if vector from facet center and A is situated on one halfplane
                                    double Aq = Vector2.Dot(A, q);
                                    if (Aq < 0.0)
                                    {
                                        // PetrelLogger.Warn("invalid ordering of facet segments");
                                        A = Vector2.Negate(A);
                                    }
                                    // check if plane is exposed to flow from source
                                    q = p - center[sourceFacet];
                                    Aq = Vector2.Dot(A, q);
                                    if (Aq > 0.0)
                                    {
                                        sourceT += Aq / Vector2.Dot(q, q);
                                    }
                                    prev = curr;
                                }
                                // compute transmissibility from target facet to source facet
                                double targetT = 0.0;
                                pos = vertex[sourceFacet];
                                prev = pos.Last();
                                foreach (Point2 curr in pos)
                                {
                                    Vector2 t = curr - prev;
                                    Point2 p = prev + 0.5 * t;
                                    Vector2 q = center[sourceFacet] - p;
                                    Vector2 s = t.ToNormalized();
                                    Vector2 n = s * Vector2.Dot(s, q) - q;
                                    Vector2 A = n.ToNormalized() * t.Norm * w_frac;
                                    // check if vector from facet center and A is situated on one halfplane
                                    double Aq = Vector2.Dot(A, q);
                                    if (Aq < 0.0)
                                    {
                                        // PetrelLogger.Warn("invalid ordering of facet segments");
                                        A = Vector2.Negate(A);
                                    }
                                    // check if plane is exposed to flow from source
                                    q = p - center[targetFacet];
                                    Aq = Vector2.Dot(A, q);
                                    if (Aq > 0.0)
                                    {
                                        targetT += Aq / Vector2.Dot(q, q);
                                    }
                                    prev = curr;
                                }
                                return 0.008527 / (1.0 / (sourcePerm * sourceT) + 1.0 / (targetPerm * targetT));
                            }


                            if (source.I + 1 == target.I && source.J == target.J && source.K == target.K)
                            {
                                // compute transmissibility multiplier in X direction
                                //
                                // compute transmissibility from source node to target facet
                                CellCorner[] sourceCorners = { CellCorner.TopSouthEast, CellCorner.TopNorthEast, CellCorner.BaseNorthEast, CellCorner.BaseSouthEast };
                                CellCorner[] targetCorners = { CellCorner.TopSouthWest, CellCorner.TopSouthWest, CellCorner.BaseNorthWest, CellCorner.BaseSouthWest };
                                double tranCell = AdjasentCellTransmissibility(sourceCorners, targetCorners, horPerm, ntg);
                                double tranFrac = FractureTransmissibility(ss[i], ss[j], m_fracPerm, m_fracPerm);
                                double tranHole = FractureTransmissibility(ss[i], ss[j],
                                    PetrelUnitSystem.ConvertToUI(horPerm.Template, horPerm[source]), PetrelUnitSystem.ConvertToUI(horPerm.Template, horPerm[target]));
                                double multTran = 1.0 + (tranFrac - tranHole) / tranCell;
                                PetrelLogger.InfoOutputWindow($"TRANX {source} -> {target} Transmissibility Multiplier = {multTran}");
                                adjacentString.AppendLine($"    \"FractureCells\" \"S{i+1}\" \"TRANSMISSIBILITY_MULTIPLIER_I\" \"{multTran}\"");
                            }
                            else if (source.I == target.I && source.J + 1 == target.J && source.K == target.K)
                            {
                                // compute transmissibility multiplier in Y direction
                                //
                                // compute transmissibility from source node to target facet
                                CellCorner[] sourceCorners = { CellCorner.TopNorthEast, CellCorner.TopNorthWest, CellCorner.BaseNorthWest, CellCorner.BaseNorthEast };
                                CellCorner[] targetCorners = { CellCorner.TopSouthEast, CellCorner.TopSouthWest, CellCorner.BaseSouthWest, CellCorner.BaseSouthEast };
                                double tranCell = AdjasentCellTransmissibility(sourceCorners, targetCorners, horPerm, ntg);
                                double tranFrac = FractureTransmissibility(ss[i], ss[j], m_fracPerm, m_fracPerm);
                                double tranHole = FractureTransmissibility(ss[i], ss[j],
                                    PetrelUnitSystem.ConvertToUI(horPerm.Template, horPerm[source]), PetrelUnitSystem.ConvertToUI(horPerm.Template, horPerm[target]));
                                double multTran = 1.0 + (tranFrac - tranHole) / tranCell;
                                PetrelLogger.InfoOutputWindow($"TRANY {source} -> {target} Transmissibility Multiplier = {multTran}");
                                adjacentString.AppendLine($"    \"FractureCells\" \"S{i + 1}\" \"TRANSMISSIBILITY_MULTIPLIER_J\" \"{multTran}\"");
                            }
                            else if (source.I == target.I && source.J == target.J && source.K + 1 == target.K)
                            {
                                // compute transmissibility multiplier in Z direction
                                //
                                // compute transmissibility from source node to target facet
                                CellCorner[] sourceCorners = { CellCorner.BaseSouthEast, CellCorner.BaseNorthEast, CellCorner.BaseNorthWest, CellCorner.BaseSouthWest };
                                CellCorner[] targetCorners = { CellCorner.TopSouthEast, CellCorner.TopNorthEast, CellCorner.TopNorthWest, CellCorner.TopSouthWest };
                                double tranCell = AdjasentCellTransmissibility(sourceCorners, targetCorners, verPerm);
                                double tranFrac = FractureTransmissibility(ss[i], ss[j], m_fracPerm, m_fracPerm);
                                double tranHole = FractureTransmissibility(ss[i], ss[j],
                                    PetrelUnitSystem.ConvertToUI(verPerm.Template, verPerm[source]), PetrelUnitSystem.ConvertToUI(verPerm.Template, verPerm[target]));
                                double multTran = 1.0 + (tranFrac - tranHole) / tranCell;
                                PetrelLogger.InfoOutputWindow($"TRANZ {source} -> {target} Transmissibility Multiplier = {multTran}");
                                adjacentString.AppendLine($"    \"FractureCells\" \"S{i + 1}\" \"TRANSMISSIBILITY_MULTIPLIER_K\" \"{multTran}\"");
                            }
                            else if (source.I + 1 == target.I && source.J + 1 == target.J && source.K == target.K)
                            {
                                //TODO: implement this case if it will be ever arised
                                PetrelLogger.InfoOutputWindow($"UNEXPECTED {source} -> {target}");
                            }
                            else
                            {
                                // non-neighbour connactions handling
                                int sourceFacet = ss[i];
                                int targetFacet = ss[j];
                                // check if segment between source and target cells intersects geometry of previously visited cells
                                Segment2 seg = new Segment2(center[sourceFacet], center[targetFacet]);
                                bool isScreened = false;
                                int k = i;
                                while (++k < j && !isScreened)
                                {
                                    int intermediate = ss[k];
                                    Point2[] pos = vertex[intermediate];
                                    //int posLength = pos.Length;
                                    //Point2 prev = pos[posLength - 1];
                                    Point2 prev = pos.Last();
                                    foreach (Point2 curr in pos)
                                    {
                                        Segment2 screen = new Segment2(prev, curr);
                                        Point2 t = seg.Intersect(screen);
                                        if (Point2.Null != t)
                                        {
                                            isScreened = true;
                                            break;
                                        }
                                        prev = curr;
                                    }
                                }
                                if (!isScreened)
                                {
                                    double tran = FractureTransmissibility(sourceFacet, targetFacet, m_fracPerm, m_fracPerm);
                                    PetrelLogger.InfoOutputWindow($"NNC {source} -> {target} Transmissibility = {tran}");
                                    distantString.AppendLine($"  ({source.I} {source.J} {source.K}) ({target.I} {target.J} {target.K}) {tran}");
                                }
                            }
                        }
                    }
                    distantString.AppendLine("]");
                    distantString.AppendLine();
                    distantString.AppendLine("ConnectionSet \"DistantConnection\" {");
                    distantString.AppendLine("  TransmissibilityBehavior=SUM");
                    distantString.AppendLine("}");
                    distantString.AppendLine();

                    adjacentString.AppendLine("]");
                    adjacentString.AppendLine();

                    fractureConnection.Text = adjacentString.ToString() + distantString.ToString();
                    transaction.Commit();
                }
            }

            if (Property.NullObject != ntg)
            {
                fieldName = "Fracture Plane";
                // IEnumerable<GeopolygonSet>
                GeopolygonSet intersectionFace = storage.GetGeopolygonSets().FirstOrDefault(item => item.Name.Equals(fieldName, StringComparison.OrdinalIgnoreCase));
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(storage);
                    if (intersectionFace != null)
                    {
                        transaction.Lock(intersectionFace);
                        intersectionFace.Delete();
                    }
                    intersectionFace = storage.CreateGeopolygonSet(fieldName);
                    transaction.Lock(intersectionFace);

                    //Matrix4 T = new Matrix4();
                    //T.Translate(-facet.Plane.DefiningPoint.X, -facet.Plane.DefiningPoint.Y, -facet.Plane.DefiningPoint.Z);
                    //Vector3 Oy = new Vector3(0.0, 1.0, 0.0);
                    //double a = Math.Acos(Vector3.Dot(Oy, facet.Plane.Normal.NormalizedVector));
                    //T.Rotate(0.0, 0.0, -a, false);

                    // PetrelLogger.InfoOutputWindow("List of geopolygons");
                    foreach (FacetCellIntersection intersection in cells)
                    {
                        if (intersection.Points.Length > 2 && ntg[intersection.CellIndex] > 0.0f)
                        {
                            Point3[] pos = (Point3[])intersection.Points.Clone();
                            T.TransformPoints(pos);
                            Geopolygon g = intersectionFace.CreateGeopolygon(new Polyline2(pos.AsEnumerable().Select(item => new Point2(item.X, item.Z))));
                            // foreach (Point3 p in pos)
                            // {
                                // PetrelLogger.InfoOutputWindow($"{p.X} {p.Y} {p.Z}");
                            // }
                            // PetrelLogger.InfoOutputWindow("---------");
                        }
                    }
                    // fieldSegment.Polylines = seg.ToArray<IPolyline3>();
                    transaction.Commit();
                }
            }
        }

        public void SetOrigin(Borehole borehole, double kickoffDepth)
        {
            m_kickoffDepth = kickoffDepth;
            TrajectoryProcessing traj = new TrajectoryProcessing(borehole.Trajectory.TrajectoryPolylineRecords);
            TrajectoryProcessing upperPart = new TrajectoryProcessing();
            TrajectoryProcessing lowerPart = new TrajectoryProcessing();
            traj.Split(m_kickoffDepth, upperPart, lowerPart);
            m_fracNode = upperPart.Trajectory().Last().Point;
        }

        public void SetShape(double leftWidth, double rightWidth, double upsideHeight, double downsideHeight, double fracAlpha, double phiStep)
        {
            m_leftWidth = leftWidth;
            m_rightWidth = rightWidth;
            m_upsideHeight = upsideHeight;
            m_downsideHeight = downsideHeight;
            m_fracAngle = fracAlpha;

            Point3 origin = m_fracNode;
            double a = (90.0 - m_fracAngle) * System.Math.PI / 180.0;
            double cosa = Math.Cos(a);
            double sina = Math.Sin(a);
            Vector3 n = new Vector3(-sina, cosa, 0.0);
            Plane3 plane = new Plane3(origin, new Direction3(n));

            List<Plane3> boundary;
            int num = (int)(90.0 / phiStep + 0.5);
            Matrix4 R = new Matrix4();
            R.Rotate(0.0, 0.0, a);
            // Matrix4 T = new Matrix4();
            Matrix4 T = R.Clone() as Matrix4;
            T.Translate(origin.X, origin.Y, origin.Z, false);
            IEnumerable<double> range = Enumerable.Range(0, num).Select(i => 0.5 * Math.PI * i / num);
            Vector3 Oy = new Vector3(0.0, 1.0, 0.0);
            IEnumerable<Point3> posLeftUpper = range.Select(phi => new Point3(-m_leftWidth * Math.Cos(phi), 0.0, m_upsideHeight * Math.Sin(phi)));
            IEnumerable<Vector3> dirLeftUpper = range.Select(phi => new Vector3(m_leftWidth * Math.Sin(phi), 0.0, m_upsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy));
            IEnumerable<Point3> posRightUpper = range.Select(phi => 0.5 * Math.PI + phi).Select(phi => new Point3(-m_rightWidth * Math.Cos(phi), 0.0, m_upsideHeight * Math.Sin(phi)));
            IEnumerable<Vector3> dirRightUpper = range.Select(phi => 0.5 * Math.PI + phi).Select(phi => new Vector3(m_rightWidth * Math.Sin(phi), 0.0, m_upsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy));
            IEnumerable<Point3> posRightLower = range.Select(phi => Math.PI + phi).Select(phi => new Point3(-m_rightWidth * Math.Cos(phi), 0.0, m_downsideHeight * Math.Sin(phi)));
            IEnumerable<Vector3> dirRightLower = range.Select(phi => Math.PI + phi).Select(phi => new Vector3(m_rightWidth * Math.Sin(phi), 0.0, m_downsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy));
            IEnumerable<Point3> posLeftLower = range.Select(phi => 1.5 * Math.PI + phi).Select(phi => new Point3(-m_leftWidth * Math.Cos(phi), 0.0, m_downsideHeight * Math.Sin(phi)));
            IEnumerable<Vector3> dirLeftLower = range.Select(phi => 1.5 * Math.PI + phi).Select(phi => new Vector3(m_leftWidth * Math.Sin(phi), 0.0, m_downsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy)).ToArray();
            Point3[] pos = posLeftUpper.Concat(posRightUpper).Concat(posRightLower).Concat(posLeftLower).ToArray();
            Vector3[] dir = dirLeftUpper.Concat(dirRightUpper).Concat(dirRightLower).Concat(dirLeftLower).ToArray();
            // R.TransformPoints(pos);
            T.TransformPoints(pos);
            R.TransformVectors(dir);
            boundary = pos.Zip(dir, (p, d) => new Plane3(p, new Direction3(-d))).ToList();

            m_fracFacet = new Facet(plane, boundary);
        }

        public void DefineFracture_FacetIntersection(string wellspec)
        {
            // fracture parameters
            //double kickoffDepth = 3970.0;
            //double downsideHeight = 60.0;
            //double upsideHeight = 90.0;
            //double leftWidth = 95.0;
            //double rightWidth = 95.0;
            //double fracAlpha = -10.0;

            // 1. Find kik-off point at specific wellname
            Borehole found = GetBorehole(wellspec);
            if (found == Borehole.NullObject)
            {
                PetrelLogger.InfoOutputWindow("cannot locate well instance");
                return;
            }

            PetrelLogger.InfoOutputWindow($"processing borehole {wellspec}");
            TrajectoryProcessing traj = new TrajectoryProcessing(found.Trajectory.TrajectoryPolylineRecords);
            TrajectoryProcessing upperPart = new TrajectoryProcessing();
            TrajectoryProcessing lowerPart = new TrajectoryProcessing();
            traj.Split(m_kickoffDepth, upperPart, lowerPart);
            //PetrelLogger.InfoOutputWindow("content of upper part of trajectory:");
            //foreach (TrajectoryPolylineRecord rec in upperPart.Trajectory()) { PetrelLogger.InfoOutputWindow($"\t{rec.MD} {rec.Point.X} {rec.Point.Y} {rec.Point.Z}"); }
            //PetrelLogger.InfoOutputWindow("content of lower part of trajectory:");
            //foreach (TrajectoryPolylineRecord rec in lowerPart.Trajectory()) { PetrelLogger.InfoOutputWindow($"\t{rec.MD} {rec.Point.X} {rec.Point.Y} {rec.Point.Z}"); }

            // 2. Create facet defining fracture intersection
            BoreholeCollection storage = ResetOutput("Ocean Labs");
            Point3 origin = upperPart.Trajectory().Last().Point;
            double a = (90.0 - m_fracAngle) * System.Math.PI / 180.0;
            double cosa = Math.Cos(a);
            double sina = Math.Sin(a);
            Vector3 n = new Vector3(-sina, cosa, 0.0);
            Plane3 plane = new Plane3(origin, new Direction3(n));

            // normal will be directed to kick-off point
            List<Plane3> boundingbox = new List<Plane3>();
            boundingbox.Add(new Plane3(new Point3(origin.X - cosa * m_leftWidth, origin.Y - sina * m_leftWidth, origin.Z), new Direction3(cosa, sina, 0.0)));
            boundingbox.Add(new Plane3(new Point3(origin.X, origin.Y, origin.Z + m_upsideHeight), new Direction3(0.0, 0.0, -1.0)));
            boundingbox.Add(new Plane3(new Point3(origin.X + cosa * m_rightWidth, origin.Y + sina * m_rightWidth, origin.Z), new Direction3(-cosa, -sina, 0.0)));
            boundingbox.Add(new Plane3(new Point3(origin.X, origin.Y, origin.Z - m_downsideHeight), new Direction3(0.0, 0.0, 1.0)));

#if MAKESQUAREPLANE
            boundary.Add(new Plane3(new Point3(origin.X - cosa * m_halfWidth, origin.Y - sina * m_halfWidth, origin.Z), new Direction3(cosa, sina, 0.0)));
            boundary.Add(new Plane3(new Point3(origin.X, origin.Y, origin.Z + m_upsideHeight), new Direction3(0.0, 0.0, -1.0)));
            boundary.Add(new Plane3(new Point3(origin.X + cosa * m_halfWidth, origin.Y + sina * m_halfWidth, origin.Z), new Direction3(-cosa, -sina, 0.0)));
            boundary.Add(new Plane3(new Point3(origin.X, origin.Y, origin.Z - m_downsideHeight), new Direction3(0.0, 0.0, 1.0)));
#else
            List<Plane3> boundary;
            {
                double phiStep = 10.0;
                int num = (int)(90.0 / phiStep + 0.5);
                Matrix4 R = new Matrix4();
                R.Rotate(0.0, 0.0, a);
                // Matrix4 T = new Matrix4();
                Matrix4 T = R.Clone() as Matrix4;
                T.Translate(origin.X, origin.Y, origin.Z, false);
                IEnumerable<double> range = Enumerable.Range(0, num).Select(i => 0.5 * Math.PI * i / num);
                Vector3 Oy = new Vector3(0.0, 1.0, 0.0);
                IEnumerable<Point3> posLeftUpper = range.Select(phi => new Point3(-m_leftWidth * Math.Cos(phi), 0.0, m_upsideHeight * Math.Sin(phi)));
                IEnumerable<Vector3> dirLeftUpper = range.Select(phi => new Vector3(m_leftWidth * Math.Sin(phi), 0.0, m_upsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy));
                IEnumerable<Point3> posRightUpper = range.Select(phi => 0.5 * Math.PI + phi).Select(phi => new Point3(-m_rightWidth * Math.Cos(phi), 0.0, m_upsideHeight * Math.Sin(phi)));
                IEnumerable<Vector3> dirRightUpper = range.Select(phi => 0.5 * Math.PI + phi).Select(phi => new Vector3(m_rightWidth * Math.Sin(phi), 0.0, m_upsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy));
                IEnumerable<Point3> posRightLower = range.Select(phi => Math.PI + phi).Select(phi => new Point3(-m_rightWidth * Math.Cos(phi), 0.0, m_downsideHeight * Math.Sin(phi)));
                IEnumerable<Vector3> dirRightLower = range.Select(phi => Math.PI + phi).Select(phi => new Vector3(m_rightWidth * Math.Sin(phi), 0.0, m_downsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy));
                IEnumerable<Point3> posLeftLower = range.Select(phi => 1.5 * Math.PI + phi).Select(phi => new Point3(-m_leftWidth * Math.Cos(phi), 0.0, m_downsideHeight * Math.Sin(phi)));
                IEnumerable<Vector3> dirLeftLower = range.Select(phi => 1.5 * Math.PI + phi).Select(phi => new Vector3(m_leftWidth * Math.Sin(phi), 0.0, m_downsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy)).ToArray();
                Point3[] pos = posLeftUpper.Concat(posRightUpper).Concat(posRightLower).Concat(posLeftLower).ToArray();
                Vector3[] dir = dirLeftUpper.Concat(dirRightUpper).Concat(dirRightLower).Concat(dirLeftLower).ToArray();
                // R.TransformPoints(pos);
                T.TransformPoints(pos);
                R.TransformVectors(dir);
                boundary = pos.Zip(dir, (p, d) => new Plane3(p, new Direction3(-d))).ToList();
                // boundary = Enumerable.Range(0, 4*num).Select(i => new Plane3(pos[i], new Direction3(dir[i]))).ToList();
                /*
                for (int i = 0; i < num; ++i)
                {
                    double phi = 0.5 * Math.PI + 0.5 * Math.PI * i / num;
                    double x = -rightWidth * Math.Cos(phi);
                    double y = upsideHeight * Math.Sin(phi);
                    double tx = rightWidth * Math.Sin(phi);
                    double ty = upsideHeight * Math.Cos(phi);
                    Point3 curr = new Point3(origin.X + cosa * x, origin.Y + sina * x, origin.Z + y);
                    Vector3 t = new Vector3(cosa * tx, sina * tx, ty);
                    Direction3 d = new Direction3(Vector3.Cross(t.ToNormalized(), n));
                    PetrelLogger.InfoOutputWindow(d.ToString());
                    Plane3 p = new Plane3(curr, d);
                    //if (p.IsInFront(origin))
                    //{
                    //    p = Plane3.Negate(p);
                    //}
                    boundary.Add(p);
                    // prev = curr;
                }
                */
            }
#endif

            // Facet facet = new Facet(plane, boundingbox);
            Facet facet = new Facet(plane, boundary);

            // PillarGridRoot root = PillarGridRoot.Get(PetrelProject.PrimaryProject);
            // GridModelRoot root = GridModelRoot.Get(PetrelProject.PrimaryProject);
            Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid grid = Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid.NullObject;
#if USEDYNAMICTYPING
            ModelCollection models = ModelCollection.NullObject;
            foreach (ModelCollection item in PetrelProject.PrimaryProject.ModelCollections)
            {
                if (item.Name.Equals("4HDM",StringComparison.OrdinalIgnoreCase))
                {
                    models = item;
                    break;
                }
            }
            if (models == ModelCollection.NullObject)
            {
                return;
            }
            foreach (Slb.Ocean.Petrel.DomainObject.Model item in models.Models)
            {
                Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid model = item as Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid;
                if (model != Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid.NullObject && model.Name.Equals("G264", StringComparison.OrdinalIgnoreCase))
                {
                    grid = model;
                    break;
                }
            }
#else
            PillarGridRoot root = PillarGridRoot.Get(PetrelProject.PrimaryProject);
            foreach (Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid item in root.Grids)
            {
                if (item.Name.Equals("G264", StringComparison.OrdinalIgnoreCase) && item.ModelCollection.Name.Equals("4HDM", StringComparison.OrdinalIgnoreCase))
                {
                    grid = item;
                    break;
                }
            }
#endif

            if (grid == Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid.NullObject)
            {
                PetrelLogger.InfoOutputWindow("cannot locate grid object");
                return;
            }

            IPillarGridIntersectionService pillarGridIntersectionService = CoreSystem.GetService<IPillarGridIntersectionService>();
            IEnumerable<FacetCellIntersection> cells = pillarGridIntersectionService.GetPillarGridPlaneIntersection(grid, facet);
            Dictionary<TrajectoryProcessing, SegmentCellIntersection> seg = traj.ComputeSegmentation(pillarGridIntersectionService.GetPillarGridPolylineIntersections(grid, found.Trajectory.TrajectoryPolyline));

            /*
            using (StreamWriter logfile = new StreamWriter("D:\\consulting\\GPN-Yamburg\\04_GEO\\ARodin\\2024-10-31\\20250604_segmentation.log"))
            {
                // Index3 gridSize = grid.NumCellsIJK;
                foreach (KeyValuePair<TrajectoryProcessing, SegmentCellIntersection> item in seg)
                {
                    TrajectoryProcessing processing = item.Key;
                    SegmentCellIntersection cellIntersection = item.Value;
                    if (cellIntersection.EnteringCell != null)
                    {
                        Index3 cellIndex = cellIntersection.EnteringCell;
                        double start = processing.Trajectory().First().MD;
                        double final = processing.Trajectory().Last().MD;
                        Index3 modelIndex = ModelingUnitSystem.ConvertIndexToUI(grid, cellIndex);
                        logfile.WriteLine($"Cell intersection ({modelIndex.I}, {modelIndex.J}, {modelIndex.K}) at interval [{start}, {final}]");
                    }
                    else
                    {
                        double start = processing.Trajectory().First().MD;
                        double final = processing.Trajectory().Last().MD;
                        logfile.WriteLine($"Outside grid interval [{start}, {final}]");
                    }
                }
            }
            */

            LocalGridSet localGridSet = grid.LocalGridSets.FirstOrDefault(item => item.Name.Equals("Local grid set 1", StringComparison.CurrentCultureIgnoreCase));
            if (LocalGridSet.NullObject != localGridSet)
            {
                LocalGrid localGrid = localGridSet.LocalGrids.FirstOrDefault(item => item.Name.Equals(found.Name, StringComparison.CurrentCultureIgnoreCase));
                if (LocalGrid.NullObject != localGrid)
                {
                    ILocalGridIntersectionService localGridIntersectionService = CoreSystem.GetService<ILocalGridIntersectionService>();
                    IEnumerable<FacetCellIntersection> voxel = localGridIntersectionService.GetLocalGridPlaneIntersection(localGrid, facet);

                    Property hostHorPerm = grid.Properties.FirstOrDefault(item => item.Name.Equals("PERMXY", StringComparison.OrdinalIgnoreCase));
                    Property hostVerPerm = grid.Properties.FirstOrDefault(item => item.Name.Equals("PERMZ", StringComparison.OrdinalIgnoreCase));
                    Property hostNtG = grid.Properties.FirstOrDefault(item => item.Name.Equals("NTG", StringComparison.OrdinalIgnoreCase));

                    LocalProperty refinedHorPerm = localGrid.GetProperty(hostHorPerm);
                    LocalProperty refinedVerPerm = localGrid.GetProperty(hostVerPerm);
                    LocalProperty refinedNtG = localGrid.GetProperty(hostNtG);

                    using (StreamWriter logfile = new StreamWriter("D:\\consulting\\GPN-Yamburg\\04_GEO\\ARodin\\2024-10-31\\20250616_segmentation.log"))
                    {
                        Index3 localGridSize = localGrid.NumCellsIJK;
                        logfile.WriteLine($"local grid size = {localGridSize}");
                        logfile.WriteLine($"number of voxels is {voxel.Count()}");
                        foreach (FacetCellIntersection face in voxel)
                        {
                            Index3 gridIndex = localGrid.IndexToGridIndex(face.CellIndex);
                            logfile.WriteLine($"voxel {face.CellIndex},  cell {gridIndex}");
                            if (face.Points.Count() < 3)
                            {
                                logfile.WriteLine("\tdegenerate polygon");
                            }
                            else if (refinedNtG[face.CellIndex] <= 0.0)
                            {
                                logfile.WriteLine("\tinactive cell");
                            }
                            else
                            {
                                double kx = PetrelUnitSystem.ConvertToUI(refinedHorPerm.Template, refinedHorPerm[face.CellIndex]);
                                double kz = PetrelUnitSystem.ConvertToUI(refinedVerPerm.Template, refinedVerPerm[face.CellIndex]);
                                logfile.WriteLine($"refined horizontal permeability = {kx}, refined vertical permeability = {kz}");
                                kx = PetrelUnitSystem.ConvertToUI(hostHorPerm.Template, hostHorPerm[gridIndex]);
                                kz = PetrelUnitSystem.ConvertToUI(hostVerPerm.Template, hostVerPerm[gridIndex]);
                                logfile.WriteLine($"host cell horizontal permeability = {kx}, host cell vertical permeability = {kz}");
                                foreach (Point3 vertex in face.Points)
                                {
                                    logfile.WriteLine($"\t{vertex}");
                                }
                            }
                        }
                    }
                    // CreatePointSet(facet, voxel, localGrid);
                }

            }

            /*
            Dictionary<int, List<FacetCellIntersection>> col = new Dictionary<int, List<FacetCellIntersection>>();
            Dictionary<int, List<FacetCellIntersection>> row = new Dictionary<int, List<FacetCellIntersection>>();
            foreach (FacetCellIntersection item in cells)
            {
                int s = item.CellIndex.I + grid.NumPillarsIJ.I * item.CellIndex.J;
                int k = item.CellIndex.K;
                if (!col.ContainsKey(s))
                {
                    col.Add(s, new List<FacetCellIntersection>());
                }
                col[s].Add(item);

                if (!row.ContainsKey(k))
                {
                    row.Add(k, new List<FacetCellIntersection>());
                }
                row[k].Add(item);
            }

            Property ntg = grid.Properties.FirstOrDefault(item => item.Name.Equals("NTG", StringComparison.OrdinalIgnoreCase));
            if (Property.NullObject != ntg)
            {
                foreach (KeyValuePair<int, List<FacetCellIntersection>> entry in col)
                {
                    entry.Value.Sort((lhs, rhs) => lhs.CellIndex.K.CompareTo(rhs.CellIndex.K));
                    // find first non-empty cell
                    // PetrelLogger.InfoOutputWindow($"iterating column {entry.Key}");
                    int prev = -1;
                    foreach (FacetCellIntersection f in entry.Value)
                    {
                        int k = f.CellIndex.K;
                        float g = ntg[f.CellIndex];
                        if (g > 0.0f)
                        {
                            if (prev < 0)
                            {

                            }
                        }
                        PetrelLogger.InfoOutputWindow($"ntg[{f.CellIndex}] = {ntg[f.CellIndex]}");
                    }
                }

            }
            */

            MakeConnection(facet, cells, grid);
            // XmlDocument xmlDocument = Calculate(found, grid, facet);
            // xmlDocument.Save(PetrelProject.GetProjectInfo(DataManager.DataSourceManager).ProjectFile.DirectoryName + "\\fracture.xml");
        }

        private static double GetGoldenRatio()
        {
            return (1.0 + Math.Sqrt(5.0)) / 2.0;
        }

        Dictionary<int, int> _cachedFib = new Dictionary<int, int>() { { 0, 0 }, { 1, 1 } };
        private int Fib(int number)
        {
            if (_cachedFib.ContainsKey(number))
                return _cachedFib[number];
            int sum = Fib(number - 1) + Fib(number - 2);
            _cachedFib[number] = sum;
            return sum;
        }

        public void DefineFracture_FractalTree(string wellspec)
        {
            //double kickoffDepth = 3970.0;
            //double downsideHeight = 60.0;
            //double upsideHeight = 90.0;
            //double halfWidth = 95.0;
            //double fracAlpha = -10.0;

            // 1. Find kik-off point at specific wellname
            Borehole found = GetBorehole(wellspec);
            if (found == Borehole.NullObject)
            {
                PetrelLogger.InfoOutputWindow("cannot locate well instance");
                return;
            }

            PetrelLogger.InfoOutputWindow($"processing borehole {wellspec}");
            TrajectoryProcessing traj = new TrajectoryProcessing(found.Trajectory.TrajectoryPolylineRecords);
            TrajectoryProcessing upperPart = new TrajectoryProcessing();
            TrajectoryProcessing lowerPart = new TrajectoryProcessing();
            traj.Split(m_kickoffDepth, upperPart, lowerPart);
            //PetrelLogger.InfoOutputWindow("content of upper part of trajectory:");
            //foreach (TrajectoryPolylineRecord rec in upperPart.Trajectory()) { PetrelLogger.InfoOutputWindow($"\t{rec.MD} {rec.Point.X} {rec.Point.Y} {rec.Point.Z}"); }
            //PetrelLogger.InfoOutputWindow("content of lower part of trajectory:");
            //foreach (TrajectoryPolylineRecord rec in lowerPart.Trajectory()) { PetrelLogger.InfoOutputWindow($"\t{rec.MD} {rec.Point.X} {rec.Point.Y} {rec.Point.Z}"); }

            // 2. Create folder for well output or clear its content if exists
            // BoreholeCollection storage = found.BoreholeCollection;
            BoreholeCollection storage = ResetOutput("Ocean Labs");

            // 3. create lateral wellbores representing fracture by fractal tree
            double a = (90.0 - m_fracAngle) * System.Math.PI / 180.0;
            double cosa = Math.Cos(a);
            double sina = Math.Sin(a);
            using (ITransaction transaction = DataManager.NewTransaction())
            {
                TrajectoryPolylineRecord origin = upperPart.Trajectory().Last();
                double y = sina * m_halfWidth;
                double x = cosa * m_halfWidth;
                transaction.Lock(storage);

                Borehole trunk = storage.CreateBorehole("Trunk1", found);
                trunk.UWI = "0000";
                // transaction.Commit();

                transaction.Lock(trunk.Trajectory.TrajectoryCollection);
                // XyTvdTrajectorySettings s = new XyTvdTrajectorySettings(WellKnownTrajectoryCalculationAlgorithms.MinimumCurvature, WellKnownElevationReferences.WorkingReferenceLevel);
                XyzTrajectory tr = trunk.Trajectory.TrajectoryCollection.CreateTrajectory<XyzTrajectory>("channel", found.Trajectory.TrajectoryCollection.WorkingTrajectory);
                // tr.Records = new[] { new XyzTrajectoryRecord(origin.Point), new XyzTrajectoryRecord(origin.Point.X+x, origin.Point.Y+y, origin.Point.Z) };
                tr.Records = new[] { new XyzTrajectoryRecord(origin.Point.X+x, origin.Point.Y+y, origin.Point.Z) };
                //tr.Settings = new XyzTrajectorySettings(CalculationAlgorithmType.Linearization, origin.MD);
                tr.Settings = new XyzTrajectorySettings(CalculationAlgorithmType.Linearization);
                trunk.Trajectory.TrajectoryCollection.DefinitiveSurvey = tr;
                // transaction.Commit();

                TrajectoryInfoFactory factory = CoreSystem.GetService<TrajectoryInfoFactory>(tr);
                TrajectoryInfo info = factory.GetTrajectoryInfo(tr);
                info.TieInMD = origin.MD;

                transaction.Commit();
            }
        }

        public Polyline3 Polyline_Create()
        {
            bool PlaneIntersectionAtSinglePoint(Plane3 p0, Plane3 p1, Plane3 p2, out Vector3 intersectionPoint)
            {
                // see Graphics Gems, Vol. 1, Pg. 305
                const double EPSILON = double.Epsilon;

                Vector3 n0 = p0.Normal.NormalizedVector;
                Vector3 n1 = p1.Normal.NormalizedVector;
                Vector3 n2 = p2.Normal.NormalizedVector;

                Vector3 q0 = new Vector3(p0.DefiningPoint.X, p0.DefiningPoint.Y, p0.DefiningPoint.Z);
                Vector3 q1 = new Vector3(p1.DefiningPoint.X, p1.DefiningPoint.Y, p1.DefiningPoint.Z);
                Vector3 q2 = new Vector3(p2.DefiningPoint.X, p2.DefiningPoint.Y, p2.DefiningPoint.Z);

                var det = Vector3.Dot(Vector3.Cross(n0, n1), n2);
                if (Math.Abs(det) < EPSILON)
                {
                    intersectionPoint = Vector3.Zero;
                    return false;
                }

                intersectionPoint = (Vector3.Dot(q0, n0) * Vector3.Cross(n1, n2) + Vector3.Dot(q1, n1) * Vector3.Cross(n2, n0) + Vector3.Dot(q2, n2) * Vector3.Cross(n0, n1)) / det;

                return true;
            }

            List<Point3> nodes = new List<Point3>();
            Facet facet = Shape;
            Plane3 prev = facet.BoundingPlanes.Last();
            foreach (Plane3 curr in facet.BoundingPlanes)
            {
                Vector3 result = Vector3.Null;
                if (PlaneIntersectionAtSinglePoint(facet.Plane, prev, curr, out result))
                {
                    nodes.Add(new Point3(result.X, result.Y, result.Z));
                }
                prev = curr;
            }

            return new Polyline3(nodes, true);
        }

        public double ShapeAngle
        {
            get
            {
                Vector3 Oy = new Vector3(0.0, 1.0, 0.0);
                return Math.Acos(Vector3.Dot(Oy, Shape.Plane.Normal.NormalizedVector));
            }
        }

        public Matrix4 WorldToScene()
        {
            // affine transform from 3D world coordinates system to 2D local coordinate system
            Point3 origin = Shape.Plane.DefiningPoint;
            Matrix4 result = new Matrix4();
            result.Translate(-origin.X, -origin.Y, -origin.Z);
            result.Rotate(0.0, 0.0, -ShapeAngle, false);
            return result;
        }

        public Matrix4 SceneToWorld()
        {
            // affine transform from 2D local coordinates to 3D world coordinate system
            Point3 origin = Shape.Plane.DefiningPoint;
            Matrix4 result = new Matrix4();
            result.Rotate(0.0, 0.0, ShapeAngle);
            result.Translate(origin.X, origin.Y, origin.Z, false);
            return result;
        }
    }
}