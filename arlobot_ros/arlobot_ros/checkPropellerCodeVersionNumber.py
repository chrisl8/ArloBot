#!/usr/bin/env python

import os


def checkPropellerCodeVersionNumber():
    propellerCodeVersionNumber = 256  # Higher than possible in the C source.

    configFilename = (
        os.path.dirname(os.path.realpath(__file__))
        + "/../../PropellerCodeForArloBot/ROSInterfaceForArloBot/include/versionNumber.h"
    )

    f = open(configFilename, "r")
    if f.mode == "r":
        contents = f.readlines()
        for line in contents:
            if "#define PROPELLER_CODE_VERSION_NUMBER" in line:
                propellerCodeVersionNumber = line.split(" ")[2].strip()

    return int(propellerCodeVersionNumber)


if __name__ == "__main__":
    result = checkPropellerCodeVersionNumber()
    print(result)
