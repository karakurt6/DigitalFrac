#define USELOCALGRIDREFINEMENT

using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using Slb.Ocean.Basics;
using Slb.Ocean.Core;
using Slb.Ocean.Geometry;
using Slb.Ocean.Petrel;
using Slb.Ocean.Petrel.DomainObject;
using Slb.Ocean.Petrel.DomainObject.Intersect;
using Slb.Ocean.Petrel.DomainObject.PillarGrid;
using Slb.Ocean.Petrel.DomainObject.Shapes;
using Slb.Ocean.Petrel.DomainObject.Well;
using Slb.Ocean.Petrel.UI;
using Slb.Ocean.Petrel.Workflow;

namespace DigitalFrac
{
    /// <summary>
    /// This class contains all the methods and subclasses of the RefinementWorkstep.
    /// Worksteps are displayed in the workflow editor.
    /// </summary>
    class RefinementWorkstep : Workstep<RefinementWorkstep.Arguments>, IExecutorSource, IAppearance, IDescriptionSource
    {
        #region Overridden Workstep methods

        /// <summary>
        /// Creates an empty Argument instance
        /// </summary>
        /// <returns>New Argument instance.</returns>

        protected override RefinementWorkstep.Arguments CreateArgumentPackageCore(IDataSourceManager dataSourceManager)
        {
            return new Arguments(dataSourceManager);
        }
        /// <summary>
        /// Copies the Arguments instance.
        /// </summary>
        /// <param name="fromArgumentPackage">the source Arguments instance</param>
        /// <param name="toArgumentPackage">the target Arguments instance</param>
        protected override void CopyArgumentPackageCore(Arguments fromArgumentPackage, Arguments toArgumentPackage)
        {
            DescribedArgumentsHelper.Copy(fromArgumentPackage, toArgumentPackage);
        }

        /// <summary>
        /// Gets the unique identifier for this Workstep.
        /// </summary>
        protected override string UniqueIdCore
        {
            get
            {
                return "DigitalFrac.RefinementWorkstep";
            }
        }
        #endregion

        #region IExecutorSource Members and Executor class

        /// <summary>
        /// Creates the Executor instance for this workstep. This class will do the work of the Workstep.
        /// </summary>
        /// <param name="argumentPackage">the argumentpackage to pass to the Executor</param>
        /// <param name="workflowRuntimeContext">the context to pass to the Executor</param>
        /// <returns>The Executor instance.</returns>
        public Slb.Ocean.Petrel.Workflow.Executor GetExecutor(object argumentPackage, WorkflowRuntimeContext workflowRuntimeContext)
        {
            return new Executor(argumentPackage as Arguments, workflowRuntimeContext);
        }

        private class ViewModel
        {
            public Borehole Well { get; set; }
            public Model.FracFacet Port { get; set; }
            public View.InputCollection Input { get; set; }
            public View.UserEdit Edit { get; set; }
            public Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid CoarseGrid { get; set; }
            public LocalGrid FineGrid { get; set; }
            public IEnumerable<Model.FracConnection> Connection { get; set; }

            public ViewModel()
            {
                Port = new Model.FracFacet();
                Input = new View.InputCollection();
                Edit = new View.UserEdit();
                Well = View.InputCollection.Borehole_Lookup("DRILL\\264_ACh_completed");
                Port.SetOrigin(Well, 0.5 * (3975.0 + 3980.0));
                Port.SetShape(95.0, 95.0, 90.0, 60.0, -10.0, 10.0);
                CoarseGrid = View.InputCollection.Grid_Lookup("4HDM", "G264");
                FineGrid = CoarseGrid.LocalGridSets.FirstOrDefault(item => item.Name.Equals("Local grid set 1", StringComparison.CurrentCultureIgnoreCase)).LocalGrids.FirstOrDefault(item => item.Name.Equals(Well.Name, StringComparison.CurrentCultureIgnoreCase));
                Model.RefinementProcessing processing = new Model.RefinementProcessing(FineGrid, "PERMXY", "PERMXY", "PERMZ", "NTG");
                Model.FracConnection.FacetFactory factory = new Model.FracConnection.FacetFactory(Port, processing, processing, processing);
                Connection = processing.GetIntersection(Port.Shape).Where(item => factory.IsValid(item)).Select(item => factory.Make(item));
            }

            public void DefineInputStage()
            {
                PolylineSet polylineSet = Input.PolylineSet_Create("FracFacet");
                Polyline3 boundary = Port.Polyline_Create();
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(polylineSet);
                    polylineSet.Polylines = new IPolyline3[] { boundary };
                    transaction.Commit();
                }

                Matrix4 T = Port.WorldToScene();
                Point3[] pos3 = boundary.ToArray();
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

                PolylinePropertyCollection attributes = View.InputCollection.PolylinePropertyCollection_Create(polylineSet);
                PolylineProperty propLabelX = View.InputCollection.WellKnownPolylineProperty_Create(attributes, WellKnownPolylinePropertyTypes.LabelX, "Label X");
                PolylineProperty propLabelY = View.InputCollection.WellKnownPolylineProperty_Create(attributes, WellKnownPolylinePropertyTypes.LabelY, "Label Y");
                PolylineProperty propLabelZ = View.InputCollection.WellKnownPolylineProperty_Create(attributes, WellKnownPolylinePropertyTypes.LabelZ, "Label Z");
                PolylineProperty propLabelAngle = View.InputCollection.WellKnownPolylineProperty_Create(attributes, WellKnownPolylinePropertyTypes.LabelAngle, "Label angle");
                PolylineProperty propArea = View.InputCollection.WellKnownPolylineProperty_Create(attributes, WellKnownPolylinePropertyTypes.Area, "Area");
                DictionaryPolylineProperty propNodeId = View.InputCollection.DictionaryPolylineProperty_Create(attributes, typeof(string), "NodeId");
                PolylineProperty propKickoffDepth = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.MeasuredDepth, "kickoff depth");
                PolylineProperty propUpsideHeight = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "upside height");
                PolylineProperty propDownsideHeight = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "downside heght");
                PolylineProperty propLeftWidth = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "left width");
                PolylineProperty propRightWidth = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "right width");
                PolylineProperty propOpenHole = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.LogTypesGroup.WellBoreRadius, "hole radius");
                PolylineProperty propFracPerm = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.FracturePropertyGroup.FracturePermeability, "fracture permeability");
                PolylineProperty propAperture = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.FracturePropertyGroup.FractureAperture, "fracture aperture");
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(polylineSet);
                    Polyline stage = polylineSet.GetPolyline(0);
                    Point3 origin = Port.Shape.Plane.DefiningPoint;
                    Slb.Ocean.Petrel.DomainObject.Basics.IPropertyAccess propertyAccess = stage.PropertyAccess;
                    propertyAccess.SetPropertyValue(propLabelX, origin.X);
                    propertyAccess.SetPropertyValue(propLabelY, origin.Y);
                    propertyAccess.SetPropertyValue(propLabelZ, origin.Z);
                    propertyAccess.SetPropertyValue(propLabelAngle, Port.Angle);
                    propertyAccess.SetPropertyValue(propArea, area);
                    propertyAccess.SetPropertyValue(propNodeId, Well.Droid.ToString()+$"//{Port.KickoffDepth:F2}");
                    propertyAccess.SetPropertyValue(propKickoffDepth, Port.KickoffDepth);
                    propertyAccess.SetPropertyValue(propDownsideHeight, Port.DownsideHeight);
                    propertyAccess.SetPropertyValue(propUpsideHeight, Port.UpsideHeight);
                    propertyAccess.SetPropertyValue(propLeftWidth, Port.LeftWidth);
                    propertyAccess.SetPropertyValue(propRightWidth, Port.RightWidth);
                    propertyAccess.SetPropertyValue(propOpenHole, Port.BoreholeDiameter * 0.5);
                    propertyAccess.SetPropertyValue(propFracPerm, PetrelUnitSystem.ConvertFromUI(PetrelProject.WellKnownTemplates.FracturePropertyGroup.FracturePermeability, Port.FracPerm));
                    propertyAccess.SetPropertyValue(propAperture, Port.Aperture);
                    transaction.Commit();
                }
            }

            public void DefineConnectionFace()
            {
                PolylineSet polylineSet = Input.PolylineSet_Create("FracConnection");
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(polylineSet);
                    Matrix4 T = Port.SceneToWorld();
                    List<IPolyline3> listPolyline3 = new List<IPolyline3>();
                    foreach (Model.FracConnection face in Connection)
                    {
                        Point3[] pos3 = face.Boundary.Select(item => new Point3(item.X, 0.0, item.Y)).ToArray();
                        T.TransformPoints(pos3);
                        listPolyline3.Add(new Polyline3(pos3, true));
                    }
                    polylineSet.Polylines = listPolyline3.ToArray();
                    transaction.Commit();
                }

                PolylinePropertyCollection attributes = View.InputCollection.PolylinePropertyCollection_Create(polylineSet);
                PolylineProperty propArea = View.InputCollection.WellKnownPolylineProperty_Create(attributes, WellKnownPolylinePropertyTypes.Area, "Area");
                DictionaryPolylineProperty propNodeId = View.InputCollection.DictionaryPolylineProperty_Create(attributes, typeof(string), "NodeId");
                DictionaryPolylineProperty propCellId = View.InputCollection.DictionaryPolylineProperty_Create(attributes, typeof(string), "CellId");
                PolylineProperty propCellHeight = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.CellDeltaZ, "cell height");
                PolylineProperty propSquareDrainage = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "square drainage");
                PolylineProperty propCircularDrainage = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "circular drainage");
                PolylineProperty propBlockPressureRadius = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "pressure radius");
                PolylineProperty propWingLength = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "wing length");
                PolylineProperty propPenetrationRatio = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.Fraction, "penetration ratio");
                PolylineProperty propConductivity = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.Fraction, "fracture conductivity");
                PolylineProperty propProppantNumber = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.Fraction, "proppant number");
                PolylineProperty propEquivalentRadius = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "equivalent radius");
                PolylineProperty propPseudoSkin = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "pseudo-skin");
                PolylineProperty propGridTerm = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "grid term");
                PolylineProperty propFracTerm = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "frac term");
                PolylineProperty propTransmissibility = View.InputCollection.PolylineProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "transmissibility");
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(polylineSet);
                    IEnumerator<Model.FracConnection> enumFace = Connection.GetEnumerator();
                    foreach (Polyline polyline in polylineSet.GetPolylines())
                    {
                        if (enumFace.MoveNext())
                        {
                            Model.FracConnection face = enumFace.Current;
                            Slb.Ocean.Petrel.DomainObject.Basics.IPropertyAccess propertyAccess = polyline.PropertyAccess;
                            propertyAccess.SetPropertyValue(propArea, face.Area);
                            propertyAccess.SetPropertyValue(propNodeId, Well.Droid.ToString() + $"/{Port.KickoffDepth:F2}");
                            propertyAccess.SetPropertyValue(propCellId, FineGrid.Droid.ToString() + $"/({face.Cell})");
                            propertyAccess.SetPropertyValue(propCellHeight, face.Height);
                            propertyAccess.SetPropertyValue(propSquareDrainage, face.SquareDrainage);
                            propertyAccess.SetPropertyValue(propCircularDrainage, face.CircularDrainage);
                            propertyAccess.SetPropertyValue(propBlockPressureRadius, face.BlockPressureRadius);
                            propertyAccess.SetPropertyValue(propWingLength, face.WingLength);
                            propertyAccess.SetPropertyValue(propPenetrationRatio, face.PenetrationRatio);
                            propertyAccess.SetPropertyValue(propConductivity, face.FractureConductivity);
                            propertyAccess.SetPropertyValue(propProppantNumber, face.ProppantNumber);
                            propertyAccess.SetPropertyValue(propEquivalentRadius, face.EquivalentRadius);
                            propertyAccess.SetPropertyValue(propPseudoSkin, face.PseudoSkin);
                            propertyAccess.SetPropertyValue(propGridTerm, face.GridBlockTerm);
                            propertyAccess.SetPropertyValue(propFracTerm, face.FracPlaneTerm);
                            propertyAccess.SetPropertyValue(propTransmissibility, face.Transmissibility);
                        }
                    }
                    transaction.Commit();
                }
            }

            public void DefineLocalGeopolygon()
            {
                GeopolygonSet geopolygonSet = Input.GeopolygonSet_Create("connectedpolygonset");
                GeopolygonPropertyCollection attributes = View.InputCollection.GeopolygonPropertyCollection_Create(geopolygonSet);
                GeopolygonProperty propArea = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Area, "Area");
                DictionaryGeopolygonProperty propNodeId = View.InputCollection.DictionaryGeopolygonProperty_Create(attributes, typeof(string), "NodeId");
                DictionaryGeopolygonProperty propCellId = View.InputCollection.DictionaryGeopolygonProperty_Create(attributes, typeof(string), "CellId");
                GeopolygonProperty propCellHeight = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.CellDeltaZ, "cell height");
                GeopolygonProperty propSquareDrainage = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "square drainage");
                GeopolygonProperty propCircularDrainage = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "circular drainage");
                GeopolygonProperty propBlockPressureRadius = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "pressure radius");
                GeopolygonProperty propWingLength = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "wing length");
                GeopolygonProperty propPenetrationRatio = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.Fraction, "penetration ratio");
                GeopolygonProperty propConductivity = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.Fraction, "fracture conductivity");
                GeopolygonProperty propProppantNumber = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.Fraction, "proppant number");
                GeopolygonProperty propEquivalentRadius = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.GeometricalGroup.Distance, "equivalent radius");
                GeopolygonProperty propPseudoSkin = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "pseudo-skin");
                GeopolygonProperty propGridTerm = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "grid term");
                GeopolygonProperty propFracTerm = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "frac term");
                GeopolygonProperty propTransmissibility = View.InputCollection.GeopolygonProperty_Create(attributes, PetrelProject.WellKnownTemplates.MiscellaneousGroup.General, "transmissibility");

                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(geopolygonSet);
                    foreach (Model.FracConnection face in Connection)
                    {
                        Geopolygon g = geopolygonSet.CreateGeopolygon(new Polyline2(face.Boundary));
                        IGeopolygonPropertyAccess propertyAccess = g.PropertyAccess;
                        propertyAccess.SetPropertyValue(propArea, face.Area);
                        propertyAccess.SetPropertyValue(propNodeId, Well.Droid.ToString() + $"/{Port.KickoffDepth:F2}");
                        propertyAccess.SetPropertyValue(propCellId, FineGrid.Droid.ToString() + $"/({face.Cell})");
                        propertyAccess.SetPropertyValue(propCellHeight, face.Height);
                        propertyAccess.SetPropertyValue(propSquareDrainage, face.SquareDrainage);
                        propertyAccess.SetPropertyValue(propCircularDrainage, face.CircularDrainage);
                        propertyAccess.SetPropertyValue(propBlockPressureRadius, face.BlockPressureRadius);
                        propertyAccess.SetPropertyValue(propWingLength, face.WingLength);
                        propertyAccess.SetPropertyValue(propPenetrationRatio, face.PenetrationRatio);
                        propertyAccess.SetPropertyValue(propConductivity, face.FractureConductivity);
                        propertyAccess.SetPropertyValue(propProppantNumber, face.ProppantNumber);
                        propertyAccess.SetPropertyValue(propEquivalentRadius, face.EquivalentRadius);
                        propertyAccess.SetPropertyValue(propPseudoSkin, face.PseudoSkin);
                        propertyAccess.SetPropertyValue(propGridTerm, face.GridBlockTerm);
                        propertyAccess.SetPropertyValue(propFracTerm, face.FracPlaneTerm);
                        propertyAccess.SetPropertyValue(propTransmissibility, face.Transmissibility);
                    }
                    transaction.Commit();
                }
            }

            public void DefineUserEdit()
            {
                ReservoirUserEdit reservoirUserEdit = Edit.ReservoirManagement_Create(Well.Name);

                StringBuilder stringBuilder = new StringBuilder();
                stringBuilder.AppendLine("START");
                stringBuilder.AppendLine("DATE \"14-Dec-2024 13:00:00\"");
                stringBuilder.AppendLine("#TIME 0.0");
                stringBuilder.AppendLine();
                stringBuilder.AppendLine("WellDef \"" + Well.Name + ";Tubing 1\" {");
                stringBuilder.AppendLine("    Undefined=\"False\"");
                stringBuilder.AppendLine();

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

                foreach (Model.FracConnection face in Connection)
                {
                    // Index3 modelIndex = ModelingUnitSystem.ConvertIndexToUI(CoarseGrid, FineGrid.IndexToGridIndex(face.Cell));
                    Index3 modelIndex = ModelingUnitSystem.ConvertIndexToUI(CoarseGrid, face.Cell);
                    dataTable.Rows.Add($"(\"{FineGrid.Name}\" {modelIndex.I} {modelIndex.J} {modelIndex.K})", "\"Frac 1\"", 1, "OPEN", -Port.Shape.Plane.DefiningPoint.Z, 
                        Port.KickoffDepth, Port.BoreholeDiameter * 0.5, 0.0, 1.0, face.BlockPressureRadius, face.BlockPermeability * face.Height, face.Transmissibility, "K");
                }

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

                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(reservoirUserEdit);
                    reservoirUserEdit.Text = stringBuilder.ToString();
                    transaction.Commit();
                }
            }
        }

        public class Executor : Slb.Ocean.Petrel.Workflow.Executor
        {
            Arguments arguments;
            WorkflowRuntimeContext context;

            Borehole _borehole;
            View.InputCollection _input;
            Model.FracFacet _fracFacet;
            Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid _grid;
            LocalGridSet _localGridSet;
            LocalGrid _localGrid;

            public Executor(Arguments arguments, WorkflowRuntimeContext context)
            {
                this.arguments = arguments;
                this.context = context;
            }

            private void ActionStub1()
            {
                _fracFacet = new Model.FracFacet();
                _input = new View.InputCollection();
                _borehole = View.InputCollection.Borehole_Lookup("DRILL\\264_ACh_completed");
                if (Borehole.NullObject != _borehole)
                {
                    _fracFacet.SetOrigin(_borehole, 0.5 * (3975.0 + 3980.0));
                    _fracFacet.SetShape(95.0, 95.0, 90.0, 60.0, -10.0, 10.0);
                    _input.PointSet_Create($"FracFacet\\{_borehole.Name}\\{_fracFacet.KickoffDepth}");

                    _grid = View.InputCollection.Grid_Lookup("4HDM", "G264");
                    if (Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid.NullObject != _grid)
                    {
#if USELOCALGRIDREFINEMENT
                        _localGridSet = _grid.LocalGridSets.FirstOrDefault(item => item.Name.Equals("Local grid set 1", StringComparison.CurrentCultureIgnoreCase));
                        if (LocalGridSet.NullObject != _localGridSet)
                        {
                            _localGrid = _localGridSet.LocalGrids.FirstOrDefault(item => item.Name.Equals(_borehole.Name, StringComparison.CurrentCultureIgnoreCase));
                            if (LocalGrid.NullObject != _localGrid)
                            {
                                Model.RefinementProcessing processing = new Model.RefinementProcessing(_localGrid, "PERMXY", "PERMXY", "PERMZ", "NTG");
                                Model.FracConnection.FacetFactory factory = new Model.FracConnection.FacetFactory(_fracFacet, processing, processing, processing);
                                IEnumerable<Model.FracConnection> connection = processing.GetIntersection(_fracFacet.Shape).Where(item => factory.IsValid(item)).Select(item => factory.Make(item));
                                PetrelLogger.InfoOutputWindow($"{connection.Count()} connections returned");
                                //Droid d = borehole.Droid;
                                //PetrelLogger.InfoOutputWindow($"{borehole.Droid} borehole has name {borehole.Name}");
                                //PetrelLogger.InfoOutputWindow($"{localGrid.Droid} localGrid has name {localGrid.Name}");
                                //Borehole b = DataManager.Resolve(d) as Borehole;
                                //if (b != null)
                                //{
                                //    PetrelLogger.InfoOutputWindow($"data manager borehole {b.Droid} borehole has name {b.Name}");
                                //}
                            }
                        }
#else
                        Model.GridProcessing processing = new Model.GridProcessing(_grid, "PERMXY", "PERMXY", "PERMZ", "NTG");
                        Model.FracConnection.FacetFactory factory = new Model.FracConnection.FacetFactory(_fracFacet, processing, processing, processing);
                        IEnumerable<Model.FracConnection> connection = processing.GetIntersection(_fracFacet.Shape).Where(item => factory.IsValid(item)).Select(item => factory.Make(item));
#endif
                    }
                }
            }

            public override void ExecuteSimple()
            {
                // Slb.Ocean.Petrel.Basics.IProjectInfo projectInfo = PetrelProject.GetProjectInfo(DataManager.DataSourceManager);
                // string projectFileName = System.IO.Path.GetFileName(projectInfo.ProjectFile.Name);
                // PetrelLogger.InfoBox($"Project File = {projectFileName}\nPrimary Project = {PetrelProject.PrimaryProject.Name}");
                // if (string.IsNullOrEmpty(projectFileName)) projectFileName = arguments.ProjName;
                // PetrelLogger.InfoBox($"Project File = {projectFileName}\nPrimary Project = {PetrelProject.PrimaryProject.Name}");
                // arguments.TreatmentWell = projectFileName;

                // TODO: Implement the workstep logic here.
                ViewModel viewModel = new ViewModel();
                viewModel.DefineInputStage();
                viewModel.DefineConnectionFace();
                viewModel.DefineUserEdit();
                viewModel.DefineLocalGeopolygon();
                //fracFacet.CreateFacet("DRILL\\264_ACh_completed");
                //fracFacet.DefineFracture_FacetIntersection("DRILL\\264_ACh_completed");
            }
        }

#endregion

        /// <summary>
        /// ArgumentPackage class for RefinementWorkstep.
        /// Each public property is an argument in the package.  The name, type and
        /// input/output role are taken from the property and modified by any
        /// attributes applied.
        /// </summary>
        public class Arguments : DescribedArgumentsByReflection
        {
            public Arguments()
                : this(DataManager.DataSourceManager)
            {                
            }

            public Arguments(IDataSourceManager dataSourceManager)
            {
            }

            private Borehole _treatmentWell = Borehole.NullObject;
            private string _startTimestamp = string.Empty;
            private string _finalTimestamp = string.Empty;
            private double _treatmentPosition = 0.0;
            private bool _positionAscending = false;
            private double _fracAngle = Double.NaN;
            private double _fracPermeability = 1000000.0;
            private double _fracAperture = 0.006;
            private double _upsideHeight = 90.0;
            private double _downsideHeight = 60.0;
            private double _leftWidth = 95.0;
            private double _rightWidth = 95.0;
            private int _sectorsNumber = 10;
            private int _recordNumber = 0;

            [Description("Treatment well", "Well assigned to fracture treatment")]
            public Borehole TreatmentWell
            {
                get { return this._treatmentWell; }
                set { this._treatmentWell = value; }
            }

            [Description("Start timestamp", "Start date and time for fracture treatment")]
            public string StartTimestamp
            {
                get { return this._startTimestamp; }
                set { this._startTimestamp = value; }
            }

            [Description("Final timestamp", "Final date and time for fracture treatment")]
            public string FinalTimestamp
            {
                get { return this._finalTimestamp; }
                set { this._finalTimestamp = value; }
            }

            [Description("Treatment position", "Nodal point fracture treatment")]
            public double TreatmentPosition
            {
                get { return this._treatmentPosition; }
                set { this._treatmentPosition = value; }
            }

            [Description("Position ascending", "True if origin of treatment position is borehole tail")]
            public Boolean PositionAscending
            {
                get { return this._positionAscending; }
                set { this._positionAscending = value; }
            }

            [Description("Fracture angle", "Orientation of fracture plane in respect to North direction")]
            public double FracAngle
            {
                get { return this._fracAngle; }
                set { this._fracAngle = value; }
            }

            [Description("Fracture permeability", "Effective permeability of fracture proppant")]
            public double FracPermeability
            {
                get { return this._fracPermeability; }
                set { this._fracPermeability = value; }
            }

            [Description("Fracture aperture", "Average aperture of fracture")]
            public double FracAperture
            {
                get { return this._fracAperture; }
                set { this._fracAperture = value; }
            }

            [Description("Upside height", "Fracture height in upward direction")]
            public double UpsideHeight
            {
                get { return this._upsideHeight; }
                set { this._upsideHeight = value; }
            }

            [Description("Downside heght", "Fracture height in downward direction")]
            public double DownsideHeight
            {
                get { return this._downsideHeight; }
                set { this._downsideHeight = value; }
            }

            [Description("Left width", "Fracture wing length in left direction")]
            public double LeftWidth
            {
                get { return this._leftWidth; }
                set { this._leftWidth = value; }
            }

            [Description("Right width", "Fracture wing length in right direction")]
            public double RightWidth
            {
                get { return this._rightWidth; }
                set { this._rightWidth = value; }
            }

            [Description("Sectors number", "Discretization of each elliptic quadrant approximating fracture treatment")]
            public int SactorsNumber
            {
                get { return this._sectorsNumber; }
                set { this._sectorsNumber = value; }
            }

            [Description("Record number", "Index in fracture catalog")]
            public int RecordNumber
            {
                get { return this._recordNumber; }
                set { this._recordNumber = value; }
            }


        }

        #region IAppearance Members
        public event EventHandler<TextChangedEventArgs> TextChanged;
        protected void RaiseTextChanged()
        {
            this.TextChanged?.Invoke(this, new TextChangedEventArgs(this));
        }

        public string Text
        {
            get { return Description.Name; }
            private set 
            {
                // TODO: implement set
                this.RaiseTextChanged();
            }
        }

        public event EventHandler<ImageChangedEventArgs> ImageChanged;
        protected void RaiseImageChanged()
        {
            this.ImageChanged?.Invoke(this, new ImageChangedEventArgs(this));
        }

        public System.Drawing.Bitmap Image
        {
            get { return PetrelImages.Modules; }
            private set 
            {
                // TODO: implement set
                this.RaiseImageChanged();
            }
        }
#endregion

#region IDescriptionSource Members

        /// <summary>
        /// Gets the description of the RefinementWorkstep
        /// </summary>
        public IDescription Description
        {
            get { return RefinementWorkstepDescription.Instance; }
        }

        /// <summary>
        /// This singleton class contains the description of the RefinementWorkstep.
        /// Contains Name, Shorter description and detailed description.
        /// </summary>
        public class RefinementWorkstepDescription : IDescription
        {
            /// <summary>
            /// Contains the singleton instance.
            /// </summary>
            private static RefinementWorkstepDescription instance = new RefinementWorkstepDescription();
            /// <summary>
            /// Gets the singleton instance of this Description class
            /// </summary>
            public static RefinementWorkstepDescription Instance
            {
                get { return instance; }
            }

#region IDescription Members

            /// <summary>
            /// Gets the name of RefinementWorkstep
            /// </summary>
            public string Name
            {
                get { return "RefinementWorkstep"; }
            }
            /// <summary>
            /// Gets the short description of RefinementWorkstep
            /// </summary>
            public string ShortDescription
            {
                get { return "create LGR for hydraulic fracture"; }
            }
            /// <summary>
            /// Gets the detailed description of RefinementWorkstep
            /// </summary>
            public string Description
            {
                get { return "represent hydraulic fracture surface with local grid refinement in simulation deck"; }
            }

#endregion
        }
#endregion


    }
}