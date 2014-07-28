''' Dbm based on sqlite -- Needed to support shelves

Key and values are always stored as bytes. This means that when strings are
used they are implicitly converted to the default encoding before being
stored.

Issues:

    # ??? how to coordinate with whichdb
    # ??? Any difference between blobs and text
    # ??? does default encoding affect str-->bytes or PySqlite3 always use UTF-8
    # ??? what is the correct isolation mode

https://code.google.com/p/dtella-cambridge/
https://code.google.com/p/dtella-cambridge/source/browse/branches/adc/dtella/common/state.py?spec=svn471&r=471
Code originally from http://bugs.python.org/file12933/dbsqlite.py
linked from http://bugs.python.org/issue3783

- modified 2010-04-19 by Ximin Luo:
  - add support for python2.6
  - add SQLFileShelf class to provide a Shelf interface; and tests for this

'''

__all__ = ['error', 'open']

import sqlite3
import collections
from shelve import Shelf
from operator import itemgetter
from itertools import imap

error = sqlite3.DatabaseError

class SQLhash(collections.MutableMapping):

    def __init__(self, filename=':memory:', flags='c', mode=None):
        # XXX add flag/mode handling
        #   c -- create if it doesn't exist
        #   n -- new empty
        #   w -- open existing
        #   r -- readonly

        MAKE_SHELF = 'CREATE TABLE IF NOT EXISTS shelf (key TEXT PRIMARY KEY, value TEXT NOT NULL)'
        self.conn = sqlite3.connect(filename)
        self.conn.text_factory = bytes
        self.conn.execute(MAKE_SHELF)
        self.conn.commit()

    def __len__(self):
        GET_LEN =  'SELECT COUNT(*) FROM shelf'
        return self.conn.execute(GET_LEN).fetchone()[0]

    def __bool__(self):
        GET_BOOL =  'SELECT MAX(ROWID) FROM shelf'   # returns None if count is zero
        return self.conn.execute(GET_BOOL).fetchone()[0] is not None

    def keys(self):
        return SQLhashKeysView(self)

    def values(self):
        return SQLhashValuesView(self)

    def items(self):
        return SQLhashItemsView(self)

    def __iter__(self):
        return iter(self.keys())

    def __contains__(self, key):
        HAS_ITEM = 'SELECT 1 FROM shelf WHERE key = ?'
        return self.conn.execute(HAS_ITEM, (key,)).fetchone() is not None

    def __getitem__(self, key):
        GET_ITEM = 'SELECT value FROM shelf WHERE key = ?'
        item = self.conn.execute(GET_ITEM, (key,)).fetchone()
        if item is None:
            raise KeyError(key)
        return item[0]

    def __setitem__(self, key, value):
        ADD_ITEM = 'REPLACE INTO shelf (key, value) VALUES (?,?)'
        self.conn.execute(ADD_ITEM, (key, value))
        #self.conn.commit()

    def __delitem__(self, key):
        if key not in self:
            raise KeyError(key)
        DEL_ITEM = 'DELETE FROM shelf WHERE key = ?'
        self.conn.execute(DEL_ITEM, (key,))
        #self.conn.commit()

    def update(self, items=(), **kwds):
        if isinstance(items, collections.Mapping):
            items = items.items()
        UPDATE_ITEMS = 'REPLACE INTO shelf (key, value) VALUES (?, ?)'
        self.conn.executemany(UPDATE_ITEMS, items)
        self.conn.commit()
        if kwds:
            self.update(kwds)

    def clear(self):
        CLEAR_ALL = 'DELETE FROM shelf;  VACUUM;'
        self.conn.executescript(CLEAR_ALL)
        self.conn.commit()

    def sync(self):
        if self.conn is not None:
            self.conn.commit()

    def close(self):
        if self.conn is not None:
            self.conn.commit()
            self.conn.close()
            self.conn = None

    def __del__(self):
        self.close()

class ListRepr:

    def __repr__(self):
        return repr(list(self))

class SQLhashKeysView(collections.KeysView, ListRepr):

    def __iter__(self):
        GET_KEYS = 'SELECT key FROM shelf ORDER BY ROWID'
        return imap(itemgetter(0), self._mapping.conn.cursor().execute(GET_KEYS))

class SQLhashValuesView(collections.ValuesView, ListRepr):

    def __iter__(self):
        GET_VALUES = 'SELECT value FROM shelf ORDER BY ROWID'
        return imap(itemgetter(0), self._mapping.conn.cursor().execute(GET_VALUES))

class SQLhashItemsView(collections.ValuesView, ListRepr):

    def __iter__(self):
        GET_ITEMS = 'SELECT key, value FROM shelf ORDER BY ROWID'
        return iter(self._mapping.conn.cursor().execute(GET_ITEMS))

def open(file=None, *args, **kwargs):
    if file is None:
        SQLhash()
    return SQLhash(file, *args, **kwargs)


class SQLFileShelf(Shelf, object):

    def __init__(self, filename, flag='c', protocol=None, writeback=False):
        Shelf.__init__(self, SQLhash(filename, flag), protocol, writeback)

    def __setitem__(self, key, val):
        return Shelf.__setitem__(self, str(key), val)

    def __getitem__(self, key):
        return Shelf.__getitem__(self, str(key))

    def __delitem__(self, key):
        return Shelf.__delitem__(self, str(key))


def test_SQLFileShelf(fn):

    db = SQLFileShelf(fn)
    db.clear()
    assert db == {}
    db[1] = [1,2,3]
    assert db[1] == [1,2,3]
    assert db == {"1": [1,2,3]}
    db[1].append(4)
    assert db[1] == [1,2,3]
    assert db == {"1": [1,2,3]}
    db.close()

    db = SQLFileShelf(fn, writeback=True)
    db.clear()
    assert db == {}
    db[1] = [1,2,3]
    assert db[1] == [1,2,3]
    assert db == {"1": [1,2,3]}
    db[1].append(4)
    assert db[1] == [1,2,3,4]
    assert db == {"1": [1,2,3,4]}
    db.close()


if __name__ in '__main___':
    for d in SQLhash(), SQLhash('example'):
        list(d)
        print(list(d), "start")
        d['abc'] = 'lmno'
        print(d['abc'])
        d['abc'] = 'rsvp'
        d['xyz'] = 'pdq'
        print(d.items())
        print(d.values())
        print(d.keys())
        print(list(d), 'list')
        d.update(p='x', q='y', r='z')
        print(d.items())

        del d['abc']
        try:
            print(d['abc'])
        except KeyError:
            pass
        else:
            raise Exception('oh noooo!')

        try:
            del d['abc']
        except KeyError:
            pass
        else:
            raise Exception('drat!')

        print(list(d))
        print(bool(d), True)
        d.clear()
        print(bool(d), False)
        print(list(d))
        d.update(p='x', q='y', r='z')
        print(list(d))
        d['xyz'] = 'pdq'

        print()
        d.close()

        import tempfile, os
        fn = tempfile.mkstemp()[1]
        test_SQLFileShelf(fn)
        os.remove(fn)
