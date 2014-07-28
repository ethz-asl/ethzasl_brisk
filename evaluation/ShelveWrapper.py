import shelve
import os
from dbsqlite import SQLFileShelf

class ShelveTable(object):
    def __init__(self, shelveDb, tableName):
        self._shelveDb = shelveDb
        self._tableName = tableName
        self._keyskey = "{}_keys".format(tableName)
        if shelveDb._db.has_key(self._keyskey):
            self._keys = self._shelveDb._db[self._keyskey]
        else:
            self._keys = set()
    def getCompositeKey(self,key):
        return "{0}{1:0>10}".format(self._tableName,key)
    def __getitem__(self,key):
        compKey = self.getCompositeKey(key)
        return self._shelveDb._db[compKey]
    def __delitem__(self,key):
        compKey = self.getCompositeKey(key)
        self._keys.remove(key)
        self._shelveDb._db[self._keyskey] = self._keys
        del self._shelveDb._db[compKey]
    def __setitem__(self,key,value):
        compKey = self.getCompositeKey(key)
        self._keys.add(key)
        self._shelveDb._db[self._keyskey] = self._keys
        self._shelveDb._db[compKey] = value
    def keys(self):
        return self._keys
    def has_key(self, key):
        return key in self._keys
    def clearTable(self):
        for key in self._keys:
            compKey = self.getCompositeKey(key)
            del self._shelveDb._db[compKey]
        self._keys = set()
        self._shelveDb._db[self._keyskey] = self._keys
    def sync(self):
        self._shelveDb.sync()

class ShelveDb(object):
    def __init__(self, filename, clear=False):
        if clear:
            print "Clearing database {0}".format(filename)
            if os.path.exists(filename):
                os.remove(filename)
        self._filename = filename
        self._db = SQLFileShelf(filename)
        
        self._table = self.getTable('mainTable')
    def __del__(self):
        self._db.close()
    def getTable(self, tableName):
        return ShelveTable(self,tableName)
    def __getitem__(self,key):
        return self._table[key]
    def __setitem__(self,key,value):
        self._table[key] = value
    def __delitem__(self,key):
        del self._table[key]
    def keys(self):
        return self._table.keys()
    def has_key(self,key):
        return self._table.has_key(key)
    def sync(self):
        self._db.sync()
