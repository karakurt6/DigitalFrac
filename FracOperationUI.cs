using System;
using System.Drawing;
using System.Windows.Forms;

using Slb.Ocean.Petrel.Workflow;
using Slb.Ocean.Core;

namespace DigitalFrac
{
    /// <summary>
    /// This class is the user interface which forms the focus for the capabilities offered by the process.  
    /// This often includes UI to set up arguments and interactively run a batch part expressed as a workstep.
    /// </summary>
    partial class FracOperationUI : UserControl
    {
        private FracOperation workstep;
        /// <summary>
        /// The argument package instance being edited by the UI.
        /// </summary>
        private FracOperation.Arguments args;
        /// <summary>
        /// Contains the actual underlaying context.
        /// </summary>
        private WorkflowContext context;

        public DialogResult Result
        {
            get;
            private set;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="FracOperationUI"/> class.
        /// </summary>
        /// <param name="workstep">the workstep instance</param>
        /// <param name="args">the arguments</param>
        /// <param name="context">the underlying context in which this UI is being used</param>
        public FracOperationUI(FracOperation workstep, FracOperation.Arguments args, WorkflowContext context)
        {
            InitializeComponent();

            this.workstep = workstep;
            this.args = args;
            this.context = context;

            View.OperationPresentation view = new View.OperationPresentation();
            FracOperationViewModel viewModel = new FracOperationViewModel();
            view.DataContext = viewModel;
            elementHost1.Child = view;
            viewModel.RequestClose += CloseDialog =>
            {
                Result = CloseDialog;
                // FracOperationUI form = view.Parent as FracOperationUI;
                // Close();
            };
        }
    }
}
