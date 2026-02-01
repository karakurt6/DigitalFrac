using System;

using Slb.Ocean.Core;
using Slb.Ocean.Petrel;
using Slb.Ocean.Petrel.UI;
using Slb.Ocean.Petrel.Workflow;
using Slb.Ocean.Petrel.DomainObject.Well;

namespace DigitalFrac
{
    /// <summary>
    /// This class contains all the methods and subclasses of the FracOperation.
    /// Worksteps are displayed in the workflow editor.
    /// </summary>
    class FracOperation : Workstep<FracOperation.Arguments>, IExecutorSource, IAppearance, IDescriptionSource
    {
        #region Overridden Workstep methods

        /// <summary>
        /// Creates an empty Argument instance
        /// </summary>
        /// <returns>New Argument instance.</returns>

        protected override FracOperation.Arguments CreateArgumentPackageCore(IDataSourceManager dataSourceManager)
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
                return "DigitalFrac.FracOperation";
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

        public class Executor : Slb.Ocean.Petrel.Workflow.Executor
        {
            Arguments arguments;
            WorkflowRuntimeContext context;

            public Executor(Arguments arguments, WorkflowRuntimeContext context)
            {
                this.arguments = arguments;
                this.context = context;
            }

            public override void ExecuteSimple()
            {
                // TODO: Implement the workstep logic here.
            }
        }

        #endregion

        /// <summary>
        /// ArgumentPackage class for FracOperation.
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

            private int operationType = 0;
            private int searchDirection = 0;
            private Slb.Ocean.Petrel.DomainObject.Well.Borehole boreholeInstance;
            private string timeStamp;
            private double kikoffPosition = 3977.0;
            private double upsideHeight = 90.0;
            private double downsideHeight = 60.0;
            private double leftWidth = 95.0;
            private double rightWidth = 95.0;
            private double orientationAngle = 170.0;
            private double fracPermeablility = 100000.0;
            private double fracAperture = 0.006;
            private double boreholeDiameter = 0.1556;
            private int sectorsNumber = 10;
            private int recordNumber;

            [OptionalInWorkflow]
            [Description("OperationType", "CRUD operation mnemonics")]
            public int OperationType
            {
                internal get { return this.operationType; }
                set { this.operationType = value; }
            }

            [OptionalInWorkflow]
            [Description("SearchDirection", "Search direction along borehole")]
            public int SearchDirection
            {
                internal get { return this.searchDirection; }
                set { this.searchDirection = value; }
            }

            [OptionalInWorkflow]
            [Description("BoreholeInstance", "Borehole subjected to hydraulic fracture treatment")]
            public Slb.Ocean.Petrel.DomainObject.Well.Borehole BoreholeInstance
            {
                get { return this.boreholeInstance; }
                set { this.boreholeInstance = value; }
            }

            [Description("TimeStamp", "Date and time of fracture treatment")]
            public string TimeStamp
            {
                get { return this.timeStamp; }
                set { this.timeStamp = value; }
            }

            [Description("KikoffPosition", "Kikoff position of fracture operation, m")]
            public double KikoffPosition
            {
                get { return this.kikoffPosition; }
                set { this.kikoffPosition = value; }
            }

            [Description("UpsideHeight", "Upside hight of fracture treatment")]
            public double UpsideHeight
            {
                get { return this.upsideHeight; }
                set { this.upsideHeight = value; }
            }

            [Description("DownsideHeight", "Downside height of fracture treatment")]
            public double DownsideHeight
            {
                get { return this.downsideHeight; }
                set { this.downsideHeight = value; }
            }

            [Description("LeftWidth", "Left wing length of fracture operation")]
            public double LeftWidth
            {
                get { return this.leftWidth; }
                set { this.leftWidth = value; }
            }

            [Description("RightWidth", "Right Wing length of fracture operation")]
            public double RightWidth
            {
                get { return this.rightWidth; }
                set { this.rightWidth = value; }
            }

            [Description("OrientationAngle", "Oprientation angle of fracture treatment")]
            public double OrientationAngle
            {
                get { return this.orientationAngle; }
                set { this.orientationAngle = value; }
            }

            [Description("FracPermeablility", "Fracture permeability, mD")]
            public double FracPermeablility
            {
                get { return this.fracPermeablility; }
                set { this.fracPermeablility = value; }
            }

            [Description("FracAperture", "Fracture width, m")]
            public double FracAperture
            {
                get { return this.fracAperture; }
                set { this.fracAperture = value; }
            }

            [Description("BoreholeDiameter", "diameter of borehole at kick-off position")]
            public double BoreholeDiameter
            {
                get { return this.boreholeDiameter; }
                set { this.boreholeDiameter = value; }
            }

            [Description("SectorsNumber", "Number of sectors in each quadrant")]
            public int SectorsNumber
            {
                internal get { return this.sectorsNumber; }
                set { this.sectorsNumber = value; }
            }

            [AssignToGlobalOutputVariable]
            [OptionalInWorkflow]
            [Description("RecordNumber", "Resulting index in fracture catalog, negative if not found")]
            public int RecordNumber
            {
                get { return this.recordNumber; }
                internal set { this.recordNumber = value; }
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
        /// Gets the description of the FracOperation
        /// </summary>
        public IDescription Description
        {
            get { return FracOperationDescription.Instance; }
        }

        /// <summary>
        /// This singleton class contains the description of the FracOperation.
        /// Contains Name, Shorter description and detailed description.
        /// </summary>
        public class FracOperationDescription : IDescription
        {
            /// <summary>
            /// Contains the singleton instance.
            /// </summary>
            private static FracOperationDescription instance = new FracOperationDescription();
            /// <summary>
            /// Gets the singleton instance of this Description class
            /// </summary>
            public static FracOperationDescription Instance
            {
                get { return instance; }
            }

            #region IDescription Members

            /// <summary>
            /// Gets the name of FracOperation
            /// </summary>
            public string Name
            {
                get { return "FracOperation"; }
            }
            /// <summary>
            /// Gets the short description of FracOperation
            /// </summary>
            public string ShortDescription
            {
                get { return "Hydraulic fracture catalog management"; }
            }
            /// <summary>
            /// Gets the detailed description of FracOperation
            /// </summary>
            public string Description
            {
                get { return "CRUD operations in Hydraulic Fracture catalog "; }
            }

            #endregion
        }
        #endregion

        public class UIFactory : WorkflowEditorUIFactory
        {
            /// <summary>
            /// This method creates the dialog UI for the given workstep, arguments
            /// and context.
            /// </summary>
            /// <param name="workstep">the workstep instance</param>
            /// <param name="argumentPackage">the arguments to pass to the UI</param>
            /// <param name="context">the underlying context in which the UI is being used</param>
            /// <returns>a Windows.Forms.Control to edit the argument package with</returns>
            protected override System.Windows.Forms.Control CreateDialogUICore(Slb.Ocean.Petrel.Workflow.Workstep workstep, object argumentPackage, WorkflowContext context)
            {
                return new FracOperationUI((FracOperation)workstep, (Arguments)argumentPackage, context);
            }
        }
    }
}