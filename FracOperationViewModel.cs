using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Windows.Input;
using Slb.Ocean.Data.Hosting;
using Slb.Ocean.Petrel;
using Slb.Ocean.Petrel.DomainObject.Well;
using Slb.Ocean.Petrel.UI.Wpf.Controls;

namespace DigitalFrac
{
    public class FracOperationViewModel: IDataErrorInfo
    {
        private Borehole _borehole;
        private InputValidationManifest<Borehole> _manifest;
        private ICommand _okCommand;
        private ICommand _cancelCommand;
        public event Action<DialogResult> RequestClose;

        public InputValidationManifest<Borehole> ValidateInput
        {
            get
            {
                return _manifest ?? (_manifest = new InputValidationManifest<Borehole>(ValidateBorehole, "Borehole"));
            }
        }
        private InputValidationResult ValidateBorehole(Borehole bh)
        {
            if (bh == null)
            {
                return new InputValidationResult(false, "Invalid input");
            }
            if (SearchableDropboxInput != null)
                SearchableDropboxInput.Changed -= BoreholeOnChanged;
            SearchableDropboxInput = bh;
            return new InputValidationResult(true, "Valid input");
        }
        public Borehole SearchableDropboxInput
        {
            get { return _borehole; }
            set
            {
                _borehole = value;
                if (_borehole != null)
                    _borehole.Changed += BoreholeOnChanged;
            }
        }
        private void BoreholeOnChanged(object sender, DomainObjectChangeEventArgs domainObjectChangeEventArgs)
        {
            ValidateBorehole(sender as Borehole);
        }
        private void Close(DialogResult dlgrResult)
        {
            if (dlgrResult == DialogResult.Yes ||
            dlgrResult == DialogResult.OK)
            {
                if (SearchableDropboxInput != null)
                    PetrelLogger.InfoOutputWindow("Object in the SearchableDropBox is borehole " + SearchableDropboxInput.Name);
            }
            if (dlgrResult == DialogResult.OK || dlgrResult == DialogResult.Cancel)
            {
                PetrelLogger.InfoOutputWindow("Closing dialog.\n");
                if (RequestClose != null)
                {
                    RequestClose(dlgrResult);
                }
            }
        }
        public ICommand OkCommand
        {
            get
            {
                return _okCommand ??
                (_okCommand = new RelayCommand(x => Close(DialogResult.OK)));
            }
        }
        public ICommand CancelCommand
        {
            get
            {
                return _cancelCommand ??
                (_cancelCommand = new RelayCommand(x => Close(DialogResult.Cancel)));
            }
        }

        public string Error
        {
            get
            {
                if (SearchableDropboxInput == null)
                {
                    return "The borehole must be selected";
                }
                return null;
            }
        }

        public string this[string columnName]
        {
            get
            {
                if (columnName == "SearchableDropboxInput")
                {
                    return "The borehole must be selected";
                }
                return null;
            }
        }

    }
}
