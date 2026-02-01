using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Slb.Ocean.Core;
using Slb.Ocean.Petrel;
using Slb.Ocean.Petrel.DomainObject;
using Slb.Ocean.Petrel.DomainObject.PillarGrid;
using Slb.Ocean.Petrel.DomainObject.Shapes;
using Slb.Ocean.Petrel.DomainObject.Well;

namespace DigitalFrac.View
{
    public class InputCollection
    {
        Collection _storage;

        public InputCollection(string name = "Ocean Labs")
        {
            _storage = Collection.NullObject;
        }

        private Collection Collection_Default()
        {
            string name = "Ocean Labs";
            if (Collection.NullObject == _storage)
            {
                IEnumerable<Collection> folders = PetrelProject.PrimaryProject.Collections;
                _storage = PetrelProject.PrimaryProject.Collections.FirstOrDefault(item => item.Name.Equals(name, StringComparison.OrdinalIgnoreCase));
                if (Collection.NullObject == _storage)
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        Project project = PetrelProject.PrimaryProject;
                        transaction.Lock(project);
                        _storage = project.CreateCollection(name);
                        transaction.Commit();
                    }
                }
            }
            return _storage;
        }

        public Collection Collection_Create(string name)
        {
            Collection prev = Collection_Default();
            if (!string.IsNullOrEmpty(name))
            {
                string[] parent = name.Split('\\');
                foreach (string child in parent)
                {
                    Collection curr = prev.Collections.FirstOrDefault(sibling => sibling.Name.Equals(child, StringComparison.CurrentCultureIgnoreCase));
                    if (Collection.NullObject == curr)
                    {
                        using (ITransaction transaction = DataManager.NewTransaction())
                        {
                            transaction.Lock(prev);
                            curr = prev.CreateCollection(child);
                            transaction.Commit();
                        }
                    }
                    prev = curr;
                }
            }
            return prev;
        }

        public void Collection_Delete(string name)
        {
            if (Collection.NullObject != _storage)
            {
                if (string.IsNullOrEmpty(name))
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        Project project = PetrelProject.PrimaryProject;
                        transaction.Lock(project);
                        _storage.Delete();
                        transaction.Commit();
                    }
                    _storage = Collection.NullObject;
                }
                else
                {
                    Collection prev = _storage;
                    Collection curr = prev;
                    string[] parent = name.Split('\\');
                    foreach (string child in parent)
                    {
                        prev = curr;
                        curr = prev.Collections.FirstOrDefault(sibling => sibling.Name.Equals(child, StringComparison.CurrentCultureIgnoreCase));
                        if (Collection.NullObject == curr)
                        {
                            break;
                        }
                    }
                    if (Collection.NullObject != curr)
                    {
                        using (ITransaction transaction = DataManager.NewTransaction())
                        {
                            transaction.Lock(prev);
                            curr.Delete();
                            transaction.Commit();
                        }
                    }
                }
            }
        }

        public static Borehole Borehole_Lookup(string wellspec)
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
                PetrelLogger.InfoOutputWindow("InputCollection.GetBorehole() - cannot locate well folder");
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
                PetrelLogger.InfoOutputWindow("InputCollection.GetBorehole() - cannot locate well instance");
                return Borehole.NullObject;
            }
            return found;
        }

        public static Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid Grid_Lookup(string collectionName, string modelName)
        {
            Project proj = PetrelProject.PrimaryProject;
            ModelCollection collection = proj.ModelCollections.FirstOrDefault(item => item.Name.Equals(collectionName, StringComparison.CurrentCultureIgnoreCase));
            if (ModelCollection.NullObject != collection)
            {
                Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid grid = PillarGridRoot.Get(proj).GetGrids(collection).FirstOrDefault(item => item.Name.Equals(modelName, StringComparison.CurrentCultureIgnoreCase));
                if (Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid.NullObject != grid)
                {
                    return grid;
                }                
            }
            return Slb.Ocean.Petrel.DomainObject.PillarGrid.Grid.NullObject;
        }

        public PointSet PointSet_Create(string siteName)
        {
            string[] parts = siteName.Split('\\');
            Collection storage = Collection_Default();
            if (parts.Length > 1)
            {
                siteName = parts.Last();
                storage = Collection_Create(string.Join("\\", parts.Take(parts.Length - 1)));
            }
            PointSet pointSet = PointSet.NullObject;
            if (Collection.NullObject != storage)
            {
                pointSet = storage.PointSets.FirstOrDefault(item => item.Name.Equals(siteName, StringComparison.OrdinalIgnoreCase));
                if (PointSet.NullObject == pointSet)
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        transaction.Lock(storage);
                        pointSet = storage.CreatePointSet(siteName);
                        transaction.Commit();
                    }
                }
            }
            return pointSet;
        }

        public void PointSet_Delete(string name)
        {
            if (Collection.NullObject != _storage && !string.IsNullOrEmpty(name))
            {
                Collection storage = _storage;
                string[] parent = name.Split('\\');
                name = parent.Last();
                for (int i = 0; i < parent.Length - 1; ++i)
                {
                    storage = storage.Collections.FirstOrDefault(sibling => sibling.Name.Equals(name, StringComparison.CurrentCultureIgnoreCase));
                    if (Collection.NullObject == storage)
                    {
                        break;
                    }
                }
                if (Collection.NullObject != storage)
                {
                    PointSet pointSet = storage.PointSets.FirstOrDefault(item => item.Name.Equals(name, StringComparison.CurrentCultureIgnoreCase));
                    if (PointSet.NullObject != pointSet)
                    {
                        using (ITransaction transaction = DataManager.NewTransaction())
                        {
                            transaction.Lock(storage);
                            pointSet.Delete();
                            transaction.Commit();
                        }
                    }
                }
            }
        }
        public PolylineSet PolylineSet_Create(string siteName)
        {
            string[] parts = siteName.Split('\\');
            Collection storage = Collection_Default();
            if (parts.Length > 1)
            {
                siteName = parts.Last();
                storage = Collection_Create(string.Join("\\", parts.Take(parts.Length - 1)));
            }
            PolylineSet polylineSet = PolylineSet.NullObject;
            if (Collection.NullObject != storage)
            {
                polylineSet = storage.PolylineSets.FirstOrDefault(item => item.Name.Equals(siteName, StringComparison.OrdinalIgnoreCase));
                if (PointSet.NullObject == polylineSet)
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        transaction.Lock(storage);
                        polylineSet = storage.CreatePolylineSet(siteName);
                        transaction.Commit();
                    }
                }
            }
            return polylineSet;
        }

        public void PolylineSet_Delete(string name)
        {
            if (Collection.NullObject != _storage && !string.IsNullOrEmpty(name))
            {
                Collection storage = _storage;
                string[] parent = name.Split('\\');
                name = parent.Last();
                for (int i = 0; i < parent.Length - 1; ++i)
                {
                    storage = storage.Collections.FirstOrDefault(sibling => sibling.Name.Equals(name, StringComparison.CurrentCultureIgnoreCase));
                    if (Collection.NullObject == storage)
                    {
                        break;
                    }
                }
                if (Collection.NullObject != storage)
                {
                    PolylineSet polylineSet = storage.PolylineSets.FirstOrDefault(item => item.Name.Equals(name, StringComparison.CurrentCultureIgnoreCase));
                    if (PointSet.NullObject != polylineSet)
                    {
                        using (ITransaction transaction = DataManager.NewTransaction())
                        {
                            transaction.Lock(storage);
                            polylineSet.Delete();
                            transaction.Commit();
                        }
                    }
                }
            }
        }
        public GeopolygonSet GeopolygonSet_Create(string siteName)
        {
            string[] parts = siteName.Split('\\');
            Collection storage = Collection_Default();
            if (parts.Length > 1)
            {
                siteName = parts.Last();
                storage = Collection_Create(string.Join("\\", parts.Take(parts.Length - 1)));
            }
            GeopolygonSet geopolygonSet = null;
            if (Collection.NullObject != storage)
            {
                geopolygonSet = storage.GetGeopolygonSets().FirstOrDefault(item => item.Name.Equals(siteName, StringComparison.OrdinalIgnoreCase));
                if (geopolygonSet == null)
                {
                    using (ITransaction transaction = DataManager.NewTransaction())
                    {
                        transaction.Lock(storage);
                        geopolygonSet = storage.CreateGeopolygonSet(siteName);
                        transaction.Commit();
                    }
                }
            }
            return geopolygonSet;
        }

        public void GeopolygonSet_Delete(string name)
        {
            if (Collection.NullObject != _storage && !string.IsNullOrEmpty(name))
            {
                Collection storage = _storage;
                string[] parent = name.Split('\\');
                name = parent.Last();
                for (int i = 0; i < parent.Length - 1; ++i)
                {
                    storage = storage.Collections.FirstOrDefault(sibling => sibling.Name.Equals(name, StringComparison.CurrentCultureIgnoreCase));
                    if (Collection.NullObject == storage)
                    {
                        break;
                    }
                }
                if (Collection.NullObject != storage)
                {
                    GeopolygonSet geopolygonSet = storage.GetGeopolygonSets().FirstOrDefault(item => item.Name.Equals(name, StringComparison.CurrentCultureIgnoreCase));
                    if (geopolygonSet != null)
                    {
                        using (ITransaction transaction = DataManager.NewTransaction())
                        {
                            transaction.Lock(storage);
                            geopolygonSet.Delete();
                            transaction.Commit();
                        }
                    }
                }
            }
        }
        public static PointProperty WellknownProperty_Create(PointSet ptSet, PointSetPropertyType propType)
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

        public static PolylineProperty PolylineProperty_Create(PolylinePropertyCollection propColl, Template t, string name)
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

        public static PolylineProperty WellKnownPolylineProperty_Create(PolylinePropertyCollection propColl, PolylinePropertyType t, string name)
        {

            PolylineProperty result = PolylineProperty.NullObject;
            if (propColl.HasWellKnownProperty(t))
            {
                result = propColl.GetWellKnownProperty(t);
            }
            else
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(propColl);
                    result = propColl.CreateWellKnownProperty(t);
                    transaction.Commit();
                }
            }
            return result;
        }

        public static DictionaryPolylineProperty DictionaryPolylineProperty_Create(PolylinePropertyCollection propColl, Type t, string name)
        {
            DictionaryPolylineProperty prop = propColl.DictionaryProperties.FirstOrDefault(item => item.Name.Equals(name, StringComparison.OrdinalIgnoreCase));
            if (DictionaryPolylineProperty.NullObject == prop)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(propColl);
                    // indexProp = propColl.CreateDictionaryProperty(typeof(Int32), indexName);
                    prop = propColl.CreateDictionaryProperty(t, name);
                    transaction.Commit();
                }
            }
            return prop;
        }

        public static PolylinePropertyCollection PolylinePropertyCollection_Create(PolylineSet polylineSet)
        {

            PolylinePropertyCollection propColl = polylineSet.PropertyCollection;
            if (propColl == null)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(polylineSet);
                    propColl = polylineSet.CreatePropertyCollection();
                    transaction.Commit();
                }
            }
            return propColl;
        }

        public static GeopolygonPropertyCollection GeopolygonPropertyCollection_Create(GeopolygonSet geopolygonSet)
        {
            GeopolygonPropertyCollection result = null;
            if (geopolygonSet.HasPropertyCollection)
            {
                result = geopolygonSet.PropertyCollection;
            }
            else
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(geopolygonSet);
                    result = geopolygonSet.CreatePropertyCollection();
                    transaction.Commit();
                }
            }
            return result;
        }

        public static DictionaryGeopolygonProperty DictionaryGeopolygonProperty_Create(GeopolygonPropertyCollection propertyCollection, Type dataType, string propertyName)
        {
            DictionaryGeopolygonProperty result = propertyCollection.DictionaryProperties.FirstOrDefault(item => item.Name.Equals(propertyName, StringComparison.CurrentCultureIgnoreCase));
            if (result == null)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(propertyCollection);
                    result = propertyCollection.CreateDictionaryProperty(dataType, propertyName);
                    transaction.Commit();
                }
            }
            return result;
        }
        public static DictionaryGeopolygonProperty DictionaryGeopolygonProperty_Create(GeopolygonPropertyCollection propertyCollection, DictionaryTemplate dictionaryTemplate, string propertyName)
        {
            DictionaryGeopolygonProperty result = propertyCollection.DictionaryProperties.FirstOrDefault(item => item.Name.Equals(propertyName, StringComparison.CurrentCultureIgnoreCase));
            if (result == null)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(propertyCollection);
                    result = propertyCollection.CreateDictionaryProperty(dictionaryTemplate, propertyName);
                    transaction.Commit();
                }
            }
            return result;
        }
        public static GeopolygonProperty GeopolygonProperty_Create(GeopolygonPropertyCollection propertyCollection, Type dataType, string propertyName)
        {
            GeopolygonProperty result = propertyCollection.Properties.FirstOrDefault(item => item.Name.Equals(propertyName, StringComparison.CurrentCultureIgnoreCase));
            if (result == null)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(propertyCollection);
                    result = propertyCollection.CreateProperty(dataType, propertyName);
                    transaction.Commit();
                }
            }
            return result;
        }
        public static GeopolygonProperty GeopolygonProperty_Create(GeopolygonPropertyCollection propertyCollection, Template template, string propertyName)
        {
            GeopolygonProperty result = propertyCollection.Properties.FirstOrDefault(item => item.Name.Equals(propertyName, StringComparison.CurrentCultureIgnoreCase));
            if (result == null)
            {
                using (ITransaction transaction = DataManager.NewTransaction())
                {
                    transaction.Lock(propertyCollection);
                    result = propertyCollection.CreateProperty(template, propertyName);
                    transaction.Commit();
                }
            }
            return result;
        }
    }
}
