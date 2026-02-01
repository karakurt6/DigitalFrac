using System;
using Slb.Ocean.Core;
using Slb.Ocean.Petrel;
using Slb.Ocean.Petrel.UI;
using Slb.Ocean.Petrel.Workflow;

namespace DigitalFrac
{
    /// <summary>
    /// This class will control the lifecycle of the Module.
    /// The order of the methods are the same as the calling order.
    /// </summary>
    public class RefinementModule : IModule
    {
        private Process m_fracoperationInstance;
        private Process m_refinementworkstepInstance;
        public RefinementModule()
        {
            //
            // TODO: Add constructor logic here
            //
        }

        #region IModule Members

        /// <summary>
        /// This method runs once in the Module life; when it loaded into the petrel.
        /// This method called first.
        /// </summary>
        public void Initialize()
        {
            // TODO:  Add RefinementModule.Initialize implementation
            CoreLogger.Info("RefinementModule.Initialize");
        }

        /// <summary>
        /// This method runs once in the Module life. 
        /// In this method, you can do registrations of the not UI related components.
        /// (eg: datasource, plugin)
        /// </summary>
        public void Integrate()
        {
            // Register DigitalFrac.FracOperation
            DigitalFrac.FracOperation fracoperationInstance = new DigitalFrac.FracOperation();
            PetrelSystem.WorkflowEditor.AddUIFactory<DigitalFrac.FracOperation.Arguments>(new DigitalFrac.FracOperation.UIFactory());
            PetrelSystem.WorkflowEditor.Add(fracoperationInstance);
            m_fracoperationInstance = new Slb.Ocean.Petrel.Workflow.WorkstepProcessWrapper(fracoperationInstance);
            PetrelSystem.ProcessDiagram.Add(m_fracoperationInstance, "Ocean Labs");

            // Register DigitalFrac.RefinementWorkstep
            DigitalFrac.RefinementWorkstep refinementworkstepInstance = new DigitalFrac.RefinementWorkstep();
            PetrelSystem.WorkflowEditor.Add(refinementworkstepInstance);
            m_refinementworkstepInstance = new Slb.Ocean.Petrel.Workflow.WorkstepProcessWrapper(refinementworkstepInstance);
            PetrelSystem.ProcessDiagram.Add(m_refinementworkstepInstance, "Ocean Labs");

            // TODO:  Add RefinementModule.Integrate implementation
            CoreLogger.Info("RefinementModule.Integrate");
        }

        /// <summary>
        /// This method runs once in the Module life. 
        /// In this method, you can do registrations of the UI related components.
        /// (eg: settingspages, treeextensions)
        /// </summary>
        public void IntegratePresentation()
        {

            // TODO:  Add RefinementModule.IntegratePresentation implementation
            CoreLogger.Info("RefinementModule.IntegratePresentation");
        }

        /// <summary>
        /// This method runs once in the Module life.
        /// right before the module is unloaded. 
        /// It usually happens when the application is closing.
        /// </summary>
        public void Disintegrate()
        {
            // Unregister DigitalFrac.FracOperation
            PetrelSystem.WorkflowEditor.RemoveUIFactory<DigitalFrac.FracOperation.Arguments>();
            PetrelSystem.ProcessDiagram.Remove(m_fracoperationInstance);
            PetrelSystem.ProcessDiagram.Remove(m_refinementworkstepInstance);
            // TODO:  Add RefinementModule.Disintegrate implementation
            CoreLogger.Info("RefinementModule.Disintegrate");
        }

        #endregion

        #region IDisposable Members

        public void Dispose()
        {
            // TODO:  Add RefinementModule.Dispose implementation
        }

        #endregion

    }


}