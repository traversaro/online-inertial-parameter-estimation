#!/usr/bin/python

#Sync the required directories from the iCub tree
import shutil, errno

def copyanything(src, dst):
    try:
        shutil.copytree(src, dst)
    except OSError as exc: # python >2.5
        if exc.errno == errno.ENOTDIR:
            shutil.copy(src, dst)
        else: raise
        
def rmanything(dst):
    try:
        shutil.rmtree(dst)
    except OSError as exc:
        pass



rmanything("./iCub/contrib/src/inertiaObserver")
rmanything("./iCub/main/src/libraries/iDyn")
rmanything("./iCub/main/src/libraries/learningMachine")

copyanything("../iCub/contrib/src/inertiaObserver","./iCub/contrib/src/inertiaObserver")
copyanything("../iCub/main/src/libraries/iDyn","./iCub/main/src/libraries/iDyn")
copyanything("../iCub/main/src/libraries/learningMachine","./iCub/main/src/libraries/learningMachine")
