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
using Slb.Ocean.Petrel.DomainObject.PillarGrid;
using Slb.Ocean.Petrel.DomainObject.Intersect;
// using Slb.Ocean.Petrel.DomainObject.Basics;
// using Slb.Ocean.Petrel.DomainObject.GridModel;

namespace DigitalFrac.Model
{
    public class PillarCollection
    {
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
            // fracture parameters
            double kickoffDepth = 3970;
            double downsideHeight = 60;
            double upsideHeight = 90;
            double halfWidth = 95;
            double pillarGap = 10;
            double fracAlpha = -10;

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
            traj.Split(kickoffDepth, upperPart, lowerPart);
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
                int num = (int)Math.Round(2.0 * halfWidth / pillarGap + 0.5, MidpointRounding.AwayFromZero);
                double a = (90.0 - fracAlpha) * System.Math.PI / 180.0;
                double cosa = Math.Cos(a);
                double sina = Math.Sin(a);
                for (int i = 0; i < num; ++i)
                {
                    double x = 2.0 * halfWidth * i / (num - 1) - halfWidth;
                    double y = sina * x;
                    x = cosa * x;
                    Point3 start = new Point3(origin.X + x, origin.Y + y, origin.Z + upsideHeight);
                    Point3 final = new Point3(origin.X + x, origin.Y + y, origin.Z - downsideHeight);

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

        private static void CreatePointSet(Facet facet, IEnumerable<FacetCellIntersection> cells, Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid grid)
        {
            Property ntg = grid.Properties.FirstOrDefault(item => item.Name.Equals("NTG", StringComparison.OrdinalIgnoreCase));
            Property horPerm = grid.Properties.FirstOrDefault(item => item.Name.Equals("PERMXY", StringComparison.OrdinalIgnoreCase));
            Property verPerm = grid.Properties.FirstOrDefault(item => item.Name.Equals("PERMZ", StringComparison.OrdinalIgnoreCase));
            Index3 numCells = grid.NumCellsIJK;
            int IndexToNaturalOrder(Index3 index) => index.I - 1 + numCells.I * (index.J - 1 + numCells.J * (index.K - 1));
            Index3 NaturalOrderToIndex(int num) => new Index3(num % numCells.I + 1, (num % (numCells.I * numCells.J)) / numCells.I + 1, num / (numCells.I * numCells.J) + 1);

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
            if (DictionaryPointProperty.NullObject == indexProp)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(propColl);                    
                    // indexProp = propColl.CreateDictionaryProperty(typeof(Int32), indexName);
                    indexProp = propColl.CreateDictionaryProperty(PetrelProject.WellKnownTemplates.DiscreteFracturePropertyGroup.FracturePatchSet, indexName);
                    transaction.Commit();
                }
            }

            string layerHeight = "layer height";

            using (ITransaction transaction = DataManager.NewTransaction())
            {
                transaction.Lock(fieldSegment);
                transaction.Lock(propColl);

                IEnumerator<FacetCellIntersection> facetEnum = selectedFacet.GetEnumerator();
                foreach (Polyline p in fieldSegment.GetPolylines())
                {
                    Slb.Ocean.Petrel.DomainObject.Basics.IPropertyAccess pa = p.PropertyAccess;
                    Point3[] pos3 = p.ToArray();
                    if (facetEnum.MoveNext())
                    {
                        FacetCellIntersection fci = facetEnum.Current;
                        if (pos3.SequenceEqual(fci.Points))
                        {
                            Int32 naturalOrder = IndexToNaturalOrder(fci.CellIndex);
                            pa.SetPropertyValue(indexProp, naturalOrder);
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
                    pa.SetPropertyValue(areaProp, area * 0.5);
                }
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

                            double K_frac = 100000;
                            double FractureTransmissibility(int sourceFacet, int targetFacet, double sourcePerm, double targetPerm)
                            {
                                double w_frac = 0.02;
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
                                double tranFrac = FractureTransmissibility(ss[i], ss[j], K_frac, K_frac);
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
                                double tranFrac = FractureTransmissibility(ss[i], ss[j], K_frac, K_frac);
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
                                double tranFrac = FractureTransmissibility(ss[i], ss[j], K_frac, K_frac);
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
                                    double tran = FractureTransmissibility(sourceFacet, targetFacet, K_frac, K_frac);
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

        public void DefineFracture_FacetIntersection(string wellspec)
        {
            // fracture parameters
            double kickoffDepth = 3970.0;
            double downsideHeight = 60.0;
            double upsideHeight = 90.0;
            double leftWidth = 95.0;
            double rightWidth = 95.0;
            double fracAlpha = -10.0;

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
            traj.Split(kickoffDepth, upperPart, lowerPart);
            //PetrelLogger.InfoOutputWindow("content of upper part of trajectory:");
            //foreach (TrajectoryPolylineRecord rec in upperPart.Trajectory()) { PetrelLogger.InfoOutputWindow($"\t{rec.MD} {rec.Point.X} {rec.Point.Y} {rec.Point.Z}"); }
            //PetrelLogger.InfoOutputWindow("content of lower part of trajectory:");
            //foreach (TrajectoryPolylineRecord rec in lowerPart.Trajectory()) { PetrelLogger.InfoOutputWindow($"\t{rec.MD} {rec.Point.X} {rec.Point.Y} {rec.Point.Z}"); }

            // 2. Create facet defining fracture intersection
            BoreholeCollection storage = ResetOutput("Ocean Labs");
            Point3 origin = upperPart.Trajectory().Last().Point;
            double a = (90.0 - fracAlpha) * System.Math.PI / 180.0;
            double cosa = Math.Cos(a);
            double sina = Math.Sin(a);
            Vector3 n = new Vector3(-sina, cosa, 0.0);
            Plane3 plane = new Plane3(origin, new Direction3(n));

            // normal will be directed to kick-off point
            List<Plane3> boundingbox = new List<Plane3>();
            boundingbox.Add(new Plane3(new Point3(origin.X - cosa * leftWidth, origin.Y - sina * leftWidth, origin.Z), new Direction3(cosa, sina, 0.0)));
            boundingbox.Add(new Plane3(new Point3(origin.X, origin.Y, origin.Z + upsideHeight), new Direction3(0.0, 0.0, -1.0)));
            boundingbox.Add(new Plane3(new Point3(origin.X + cosa * rightWidth, origin.Y + sina * rightWidth, origin.Z), new Direction3(-cosa, -sina, 0.0)));
            boundingbox.Add(new Plane3(new Point3(origin.X, origin.Y, origin.Z - downsideHeight), new Direction3(0.0, 0.0, 1.0)));

#if MAKESQUAREPLANE
            boundary.Add(new Plane3(new Point3(origin.X - cosa * halfWidth, origin.Y - sina * halfWidth, origin.Z), new Direction3(cosa, sina, 0.0)));
            boundary.Add(new Plane3(new Point3(origin.X, origin.Y, origin.Z + upsideHeight), new Direction3(0.0, 0.0, -1.0)));
            boundary.Add(new Plane3(new Point3(origin.X + cosa * halfWidth, origin.Y + sina * halfWidth, origin.Z), new Direction3(-cosa, -sina, 0.0)));
            boundary.Add(new Plane3(new Point3(origin.X, origin.Y, origin.Z - downsideHeight), new Direction3(0.0, 0.0, 1.0)));
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
                IEnumerable<Point3> posLeftUpper = range.Select(phi => new Point3(-leftWidth * Math.Cos(phi), 0.0, upsideHeight * Math.Sin(phi)));
                IEnumerable<Vector3> dirLeftUpper = range.Select(phi => new Vector3(leftWidth * Math.Sin(phi), 0.0, upsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy));
                IEnumerable<Point3> posRightUpper = range.Select(phi => 0.5 * Math.PI + phi).Select(phi => new Point3(-rightWidth * Math.Cos(phi), 0.0, upsideHeight * Math.Sin(phi)));
                IEnumerable<Vector3> dirRightUpper = range.Select(phi => 0.5 * Math.PI + phi).Select(phi => new Vector3(rightWidth * Math.Sin(phi), 0.0, upsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy));
                IEnumerable<Point3> posRightLower = range.Select(phi => Math.PI + phi).Select(phi => new Point3(-rightWidth * Math.Cos(phi), 0.0, downsideHeight * Math.Sin(phi)));
                IEnumerable<Vector3> dirRightLower = range.Select(phi => Math.PI + phi).Select(phi => new Vector3(rightWidth * Math.Sin(phi), 0.0, downsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy));
                IEnumerable<Point3> posLeftLower = range.Select(phi => 1.5 * Math.PI + phi).Select(phi => new Point3(-leftWidth * Math.Cos(phi), 0.0, downsideHeight * Math.Sin(phi)));
                IEnumerable<Vector3> dirLeftLower = range.Select(phi => 1.5 * Math.PI + phi).Select(phi => new Vector3(leftWidth * Math.Sin(phi), 0.0, downsideHeight * Math.Cos(phi))).Select(t => Vector3.Cross(t.ToNormalized(), Oy)).ToArray();
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

            CreatePointSet(facet, cells, grid);
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
            double kickoffDepth = 3970.0;
            double downsideHeight = 60.0;
            double upsideHeight = 90.0;
            double halfWidth = 95.0;
            double fracAlpha = -10.0;

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
            traj.Split(kickoffDepth, upperPart, lowerPart);
            //PetrelLogger.InfoOutputWindow("content of upper part of trajectory:");
            //foreach (TrajectoryPolylineRecord rec in upperPart.Trajectory()) { PetrelLogger.InfoOutputWindow($"\t{rec.MD} {rec.Point.X} {rec.Point.Y} {rec.Point.Z}"); }
            //PetrelLogger.InfoOutputWindow("content of lower part of trajectory:");
            //foreach (TrajectoryPolylineRecord rec in lowerPart.Trajectory()) { PetrelLogger.InfoOutputWindow($"\t{rec.MD} {rec.Point.X} {rec.Point.Y} {rec.Point.Z}"); }

            // 2. Create folder for well output or clear its content if exists
            // BoreholeCollection storage = found.BoreholeCollection;
            BoreholeCollection storage = ResetOutput("Ocean Labs");

            // 3. create lateral wellbores representing fracture by fractal tree
            double a = (90.0 - fracAlpha) * System.Math.PI / 180.0;
            double cosa = Math.Cos(a);
            double sina = Math.Sin(a);
            using (ITransaction transaction = DataManager.NewTransaction())
            {
                TrajectoryPolylineRecord origin = upperPart.Trajectory().Last();
                double y = sina * halfWidth;
                double x = cosa * halfWidth;
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
    }
}