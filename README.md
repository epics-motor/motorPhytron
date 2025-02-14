# motorPhytron
EPICS motor drivers for the following [Phytron GmbH](https://www.phytron.eu/) controllers: phyMOTION I1AM01, I1AM02, I1EM01, I1EM02 or MCC-1, MCC-2 Stepper Motor Controller

[![Build Status](https://github.com/epics-motor/motorPhytron/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-motor/motorPhytron/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-motor/motorPhytron.png)](https://travis-ci.org/epics-motor/motorPhytron)-->

motorPhytron is a submodule of [motor](https://github.com/epics-modules/motor).  When motorPhytron is built in the ``motor/modules`` directory, no manual configuration is needed.

motorPhytron can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorPhytron contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.

See more documentation in [phytronApp/src/README.txt](phytronApp/src/README.txt).

SPDX-License-Identifier: EPICS
