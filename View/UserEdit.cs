using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Slb.Ocean.Core;
using Slb.Ocean.Petrel;
using Slb.Ocean.Petrel.DomainObject.Intersect;

namespace DigitalFrac.View
{
    public class UserEdit
    {
        private UserEditCollection _oceanLabUserEditColl;
        public UserEdit()
        {
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
            _oceanLabUserEditColl = topLevelUserEditColl.UserEditCollections.FirstOrDefault(item => item.Name.Equals("Ocean Labs", StringComparison.OrdinalIgnoreCase));
            // create Ocean Labs collection in INTERSECT User Edit
            if (_oceanLabUserEditColl == null)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(topLevelUserEditColl);
                    _oceanLabUserEditColl = topLevelUserEditColl.CreateUserEditCollection("Ocean Labs");
                    transaction.Commit();
                }
            }
        }

        public UserEditCollection Collection_Create(string spec)
        {
            UserEditCollection prev = _oceanLabUserEditColl;

            if (!string.IsNullOrEmpty(spec))
            {
                string[] parent = spec.Split('\\');
                foreach (string child in parent)
                {
                    UserEditCollection curr = prev.UserEditCollections.FirstOrDefault(sibling => sibling.Name.Equals(child, StringComparison.CurrentCultureIgnoreCase));
                    if (curr == null)
                    {
                        using (ITransaction transaction = DataManager.NewTransaction())
                        {
                            transaction.Lock(prev);
                            curr = prev.CreateUserEditCollection(child);
                            transaction.Commit();
                        }
                    }
                    prev = curr;
                }
            }
            return prev;
        }

        public void Collection_Delete(string spec)
        {
            if (string.IsNullOrEmpty(spec))
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(_oceanLabUserEditColl.ParentCollection);
                    _oceanLabUserEditColl.Delete();
                    transaction.Commit();
                }
                _oceanLabUserEditColl = null;
            }
            else
            {
                UserEditCollection collection = Collection_Create(spec);
                if (collection != null)
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        transaction.Lock(collection.ParentCollection);
                        collection.Delete();
                        transaction.Commit();
                    }
                }
            }
        }

        public ReservoirUserEdit ReservoirManagement_Create(string spec)
        {
            string[] parts = spec.Split('\\');
            UserEditCollection collection = _oceanLabUserEditColl;
            if (parts.Length > 1)
            {
                collection = Collection_Create(string.Join("\\", parts.Take(parts.Length - 1)));
            }
            if (collection != null)
            {
                ReservoirUserEdit reservoirUserEdit = collection.ReservoirUserEdits.FirstOrDefault(item => item.Name.Equals(parts.Last(), StringComparison.CurrentCultureIgnoreCase));
                if (reservoirUserEdit == null)
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        transaction.Lock(collection);
                        reservoirUserEdit = collection.CreateReservoirUserEdit(parts.Last());
                        transaction.Commit();
                    }
                }
                return reservoirUserEdit;
            }
            return null;
        }

        public void ReservoirManagement_Delete(string spec)
        {
            string[] parts = spec.Split('\\');
            UserEditCollection collection = _oceanLabUserEditColl;
            if (parts.Length > 1)
            {
                collection = Collection_Create(string.Join("\\", parts.Take(parts.Length - 1)));
            }
            if (collection != null)
            {
                ReservoirUserEdit reservoirUserEdit = collection.ReservoirUserEdits.FirstOrDefault(item => item.Name.Equals(parts.Last(), StringComparison.CurrentCultureIgnoreCase));
                if (reservoirUserEdit != null)
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        transaction.Lock(collection);
                        reservoirUserEdit.Delete();
                        transaction.Commit();
                    }
                }
            }
        }

        public FieldManagementUserEdit FieldManagement_Create(string spec)
        {
            string[] parts = spec.Split('\\');
            UserEditCollection collection = _oceanLabUserEditColl;
            if (parts.Length > 1)
            {
                collection = Collection_Create(string.Join("\\", parts.Take(parts.Length - 1)));
            }
            if (collection != null)
            {
                FieldManagementUserEdit fieldManagementUserEdit = collection.FieldManagementUserEdits.FirstOrDefault(item => item.Name.Equals(parts.Last(), StringComparison.CurrentCultureIgnoreCase));
                if (fieldManagementUserEdit == null)
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        transaction.Lock(collection);
                        fieldManagementUserEdit = collection.CreateFieldManagementUserEdit(parts.Last());
                        transaction.Commit();
                    }
                }
                return fieldManagementUserEdit;
            }
            return null;
        }

        void FieldManagement_Delete(string spec)
        {
            string[] parts = spec.Split('\\');
            UserEditCollection collection = _oceanLabUserEditColl;
            if (parts.Length > 1)
            {
                collection = Collection_Create(string.Join("\\", parts.Take(parts.Length - 1)));
            }
            if (collection != null)
            {
                FieldManagementUserEdit fieldManagementUserEdit = collection.FieldManagementUserEdits.FirstOrDefault(item => item.Name.Equals(parts.Last(), StringComparison.CurrentCultureIgnoreCase));
                if (fieldManagementUserEdit != null)
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        transaction.Lock(collection);
                        fieldManagementUserEdit.Delete();
                        transaction.Commit();
                    }
                }
            }
        }
    }
}
