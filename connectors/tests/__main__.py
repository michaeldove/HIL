from __future__ import print_function
import os.path
import doctest

################################################################################

def read(filename):
    directory = os.path.dirname(__file__)
    path = '%s/%s' % (directory, filename)
    contents = open(path, 'r').read()
    return contents

################################################################################

def suite():
    globs = {'read': read,
             }
    suite = doctest.DocFileSuite('simconnector.txt',
                                 optionflags=doctest.ELLIPSIS,
                                 globs=globs)
    return suite

def main():
    import unittest
    unittest.TextTestRunner().run(suite())

if __name__ == '__main__':
    main()
