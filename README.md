# motorPhytron
EPICS motor drivers for the following [Phytron GmbH](https://www.phytron.eu/) controllers: I1AM01 Stepper Motor Controller

motorPhytron is a submodule of [motor](https://github.com/epics-modules/motor).  When motorPhytron is built in the ``motor/modules`` directory, no manual configuration is needed.

motorPhytron can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorPhytron contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
